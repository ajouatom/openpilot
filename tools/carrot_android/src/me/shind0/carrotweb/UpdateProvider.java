package me.shind0.carrotweb;

import android.content.ContentProvider;
import android.content.ContentValues;
import android.database.Cursor;
import android.database.MatrixCursor;
import android.net.Uri;
import android.os.ParcelFileDescriptor;
import android.provider.OpenableColumns;
import java.io.File;
import java.io.FileNotFoundException;

/** Read-only access to exactly one verified private update APK, by temporary URI grant. */
public final class UpdateProvider extends ContentProvider {
  public boolean onCreate() { return true; }
  private File file(Uri uri) throws FileNotFoundException {
    if (!"/update.apk".equals(uri.getPath()) || uri.getQuery()!=null) throw new FileNotFoundException();
    return new File(getContext().getCacheDir(), "update.apk");
  }
  public ParcelFileDescriptor openFile(Uri uri, String mode) throws FileNotFoundException {
    if (!"r".equals(mode)) throw new FileNotFoundException("Read only");
    return ParcelFileDescriptor.open(file(uri), ParcelFileDescriptor.MODE_READ_ONLY);
  }
  public String getType(Uri uri) { return "application/vnd.android.package-archive"; }
  public Cursor query(Uri uri, String[] projection, String selection, String[] args, String sort) {
    MatrixCursor result = new MatrixCursor(new String[]{OpenableColumns.DISPLAY_NAME, OpenableColumns.SIZE});
    try { result.addRow(new Object[]{"CarrotAssist.apk",file(uri).length()}); } catch (FileNotFoundException ignored) {}
    return result;
  }
  public Uri insert(Uri uri, ContentValues values) { throw new UnsupportedOperationException(); }
  public int update(Uri uri, ContentValues values, String selection, String[] args) { throw new UnsupportedOperationException(); }
  public int delete(Uri uri, String selection, String[] args) { throw new UnsupportedOperationException(); }
}
