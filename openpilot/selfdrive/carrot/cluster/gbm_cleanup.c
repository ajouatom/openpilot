// HUD-only compatibility for the hash-verified comma raylib builds in
// cluster_graphics_cleanup.py. Never preload this into the manager or UI.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <dirent.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

struct gbm_device;
struct gbm_surface;
struct gbm_bo;

enum { MAX_DEVICES = 8, MAX_SURFACES = 16, MAX_LOCKED = 16 };
static struct device_record { struct gbm_device *ptr; int fd; bool owned; } devices[MAX_DEVICES];
static struct surface_record {
  struct gbm_surface *ptr;
  struct gbm_device *device;
  struct gbm_bo *locked[MAX_LOCKED];
} surfaces[MAX_SURFACES];
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static bool enabled, own_drm_fd;
static void *gbm_library;
static struct gbm_device *(*real_create_device)(int);
static void (*real_destroy_device)(struct gbm_device *);
static struct gbm_surface *(*real_create_surface)(struct gbm_device *, uint32_t, uint32_t, uint32_t, uint32_t);
static void (*real_destroy_surface)(struct gbm_surface *);
static struct gbm_bo *(*real_lock_buffer)(struct gbm_surface *);
static void (*real_release_buffer)(struct gbm_surface *, struct gbm_bo *);

// Called once before importing raylib or starting renderer workers.
int carrot_gbm_cleanup_enable(int owns_fd) {
  if (enabled) return own_drm_fd == (owns_fd != 0);
  gbm_library = dlopen("libgbm.so.1", RTLD_NOW | RTLD_LOCAL);
  if (!gbm_library) return 0;
  real_create_device = dlsym(gbm_library, "gbm_create_device");
  real_destroy_device = dlsym(gbm_library, "gbm_device_destroy");
  real_create_surface = dlsym(gbm_library, "gbm_surface_create");
  real_destroy_surface = dlsym(gbm_library, "gbm_surface_destroy");
  real_lock_buffer = dlsym(gbm_library, "gbm_surface_lock_front_buffer");
  real_release_buffer = dlsym(gbm_library, "gbm_surface_release_buffer");
  if (!real_create_device || !real_destroy_device || !real_create_surface ||
      !real_destroy_surface || !real_lock_buffer || !real_release_buffer) return 0;
  own_drm_fd = owns_fd != 0;
  enabled = true;
  return 1;
}

static bool raylib_caller(void *address) {
  Dl_info info;
  if (!enabled || !dladdr(address, &info) || !info.dli_fname) return false;
  const char *name = strrchr(info.dli_fname, '/');
  name = name ? name + 1 : info.dli_fname;
  return strncmp(name, "_raylib_cffi_comma.", strlen("_raylib_cffi_comma.")) == 0;
}

static void capacity_warning(void) {
  fputs("[HUD graphics] native resource tracking capacity exceeded\n", stderr);
}

static bool drm_socket(int fd) {
  struct sockaddr_un peer = {0};
  socklen_t length = sizeof(peer);
  return getpeername(fd, (struct sockaddr *)&peer, &length) == 0 && peer.sun_family == AF_UNIX &&
    length <= sizeof(peer) && memcmp(peer.sun_path, "/tmp/drmfd.sock", sizeof("/tmp/drmfd.sock")) == 0;
}

static void close_descriptor_socket(void) {
  // The verified init_drm() leaves its one-shot descriptor-passing socket open.
  // At gbm_create_device() recv_fd() has returned, so that socket is no longer in
  // use. Close only a unique matching connection in this dedicated HUD process;
  // never guess among multiple callers or close other sockets/descriptor types.
  int saved_errno = errno, candidate = -1, matches = 0;
  struct stat original = {0}, current = {0};
  DIR *directory = opendir("/proc/self/fd");
  if (directory) {
    struct dirent *entry;
    while ((entry = readdir(directory))) {
      char *end;
      long value = strtol(entry->d_name, &end, 10);
      if (*end || value < 0 || value > 0x7fffffff) continue;
      int fd = (int)value;
      if (drm_socket(fd)) {
        matches++;
        if (fstat(fd, &original) == 0) candidate = fd;
      }
    }
    closedir(directory);
  }
  if (matches == 1 && candidate >= 0 && fstat(candidate, &current) == 0 &&
      original.st_dev == current.st_dev && original.st_ino == current.st_ino && drm_socket(candidate)) close(candidate);
  errno = saved_errno;
}

struct gbm_device *gbm_create_device(int fd) {
  struct gbm_device *device = real_create_device(fd);
  if (device && raylib_caller(__builtin_return_address(0))) {
    if (own_drm_fd) close_descriptor_socket();
    pthread_mutex_lock(&mutex);
    int slot;
    for (slot = 0; slot < MAX_DEVICES; slot++) {
      if (!devices[slot].ptr) { devices[slot] = (struct device_record){device, fd, own_drm_fd}; break; }
    }
    if (slot == MAX_DEVICES) capacity_warning();
    pthread_mutex_unlock(&mutex);
  }
  return device;
}

struct gbm_surface *gbm_surface_create(struct gbm_device *device, uint32_t width, uint32_t height, uint32_t format, uint32_t flags) {
  struct gbm_surface *surface = real_create_surface(device, width, height, format, flags);
  if (!surface) return surface;
  pthread_mutex_lock(&mutex);
  bool tracked = false;
  for (int i = 0; i < MAX_DEVICES; i++) tracked |= devices[i].ptr == device;
  if (tracked) {
    int slot;
    for (slot = 0; slot < MAX_SURFACES; slot++) {
      if (!surfaces[slot].ptr) { surfaces[slot] = (struct surface_record){.ptr=surface, .device=device}; break; }
    }
    if (slot == MAX_SURFACES) capacity_warning();
  }
  pthread_mutex_unlock(&mutex);
  return surface;
}

struct gbm_bo *gbm_surface_lock_front_buffer(struct gbm_surface *surface) {
  struct gbm_bo *buffer = real_lock_buffer(surface);
  if (!buffer) return buffer;
  pthread_mutex_lock(&mutex);
  for (int i = 0; i < MAX_SURFACES; i++) {
    if (surfaces[i].ptr != surface) continue;
    int j;
    for (j = 0; j < MAX_LOCKED; j++) {
      if (!surfaces[i].locked[j]) { surfaces[i].locked[j] = buffer; break; }
    }
    if (j == MAX_LOCKED) capacity_warning();
  }
  pthread_mutex_unlock(&mutex);
  return buffer;
}

void gbm_surface_release_buffer(struct gbm_surface *surface, struct gbm_bo *buffer) {
  pthread_mutex_lock(&mutex);
  for (int i = 0; i < MAX_SURFACES; i++) {
    if (surfaces[i].ptr != surface) continue;
    for (int j = 0; j < MAX_LOCKED; j++) {
      if (surfaces[i].locked[j] == buffer) surfaces[i].locked[j] = NULL;
    }
  }
  pthread_mutex_unlock(&mutex);
  real_release_buffer(surface, buffer);
}

void gbm_surface_destroy(struct gbm_surface *surface) {
  pthread_mutex_lock(&mutex);
  for (int i = 0; i < MAX_SURFACES; i++) {
    if (surfaces[i].ptr == surface) surfaces[i] = (struct surface_record){0};
  }
  pthread_mutex_unlock(&mutex);
  real_destroy_surface(surface);
}

void gbm_device_destroy(struct gbm_device *device) {
  // raylib has already unbound/destroyed EGL before entering this function.
  // Copy and remove ownership before calling GBM; do not hold our lock in GBM.
  struct surface_record pending[MAX_SURFACES];
  int count = 0, owned_fd = -1;
  pthread_mutex_lock(&mutex);
  for (int i = 0; i < MAX_SURFACES; i++) {
    if (surfaces[i].ptr && surfaces[i].device == device) {
      pending[count++] = surfaces[i];
      surfaces[i] = (struct surface_record){0};
    }
  }
  for (int i = 0; i < MAX_DEVICES; i++) {
    if (devices[i].ptr == device) {
      if (devices[i].owned) owned_fd = devices[i].fd;
      devices[i] = (struct device_record){0};
    }
  }
  pthread_mutex_unlock(&mutex);
  for (int i = 0; i < count; i++) {
    for (int j = 0; j < MAX_LOCKED; j++) {
      if (pending[i].locked[j]) real_release_buffer(pending[i].ptr, pending[i].locked[j]);
    }
    real_destroy_surface(pending[i].ptr);
  }
  real_destroy_device(device);
  // A DRM_FD explicitly supplied by the caller is borrowed and is never closed.
  if (owned_fd >= 0) close(owned_fd);
}

