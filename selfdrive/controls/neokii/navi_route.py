#!/usr/bin/env python3
import json
import socketserver
import struct
import threading
from threading import Thread

from cereal import messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.navd.helpers import Coordinate
from openpilot.common.params import Params

ROUTE_RECEIVE_PORT = 2845

class NaviRoute():
  def __init__(self):
    self.sm = messaging.SubMaster(['managerState'])
    self.pm = messaging.PubMaster(['navInstructionNda', 'navRouteNda'])
    self.last_routes = None
    self.ui_pid = None
    self.last_client_address = None

    route_thread = Thread(target=self.route_thread, args=[])
    route_thread.daemon = True
    route_thread.start()

  def update(self):
    self.sm.update(0)

    if self.sm.updated["managerState"]:
      ui_pid = [p.pid for p in self.sm["managerState"].processes if p.name == "ui" and p.running]
      if ui_pid:
        if self.ui_pid and self.ui_pid != ui_pid[0]:
          threading.Timer(5.0, self.send_route).start()
        self.ui_pid = ui_pid[0]

  def route_thread(self):
    route_server = self.RouteTCPServer(('0.0.0.0', ROUTE_RECEIVE_PORT), self.RouteTCPHandler, self)
    route_server.serve_forever()

  def send_route(self):
    msg = messaging.new_message('navRouteNda', valid=True)
    if self.last_routes is not None:
      msg.navRouteNda.coordinates = self.last_routes
    else:
      self.dispatch_instruction(None)

    self.pm.send('navRouteNda', msg)

  def dispatch_route(self, routes):
    self.last_routes = routes
    self.send_route()

  def dispatch_instruction(self, json):
    #print("dispatch_instruction", json)
    msg = messaging.new_message('navInstructionNda', valid=True)
    instruction = msg.navInstructionNda

    if json is not None:
      if 'maneuverDistance' in json:
        instruction.maneuverDistance = float(json['maneuverDistance'])
      if 'distanceRemaining' in json:
        instruction.distanceRemaining = float(json['distanceRemaining'])
      if 'timeRemaining' in json:
        instruction.timeRemaining = float(json['timeRemaining'])
      if 'timeRemainingTypical' in json:
        instruction.timeRemainingTypical = float(json['timeRemainingTypical'])
      if 'speedLimit' in json:
        instruction.speedLimit = float(json['speedLimit'])

      if 'maneuverPrimaryText' in json:
        instruction.maneuverPrimaryText = str(json['maneuverPrimaryText'])
      if 'maneuverSecondaryText' in json:
        instruction.maneuverSecondaryText = str(json['maneuverSecondaryText'])
      if 'type' in json:
        if self.last_client_address is not None:
          instruction.imageUrl = 'http://' + self.last_client_address + ':2859/image?no=' + str(json['type'])

      maneuvers = []
      if 'maneuvers' in json:
        for m in json['maneuvers']:
          maneuver = {}
          if 'distance' in m:
            maneuver['distance'] = float(m['distance'])
          if 'type' in m:
            maneuver['type'] = m['type']
          if 'modifier' in m:
            maneuver['modifier'] = m['modifier']

          maneuvers.append(maneuver)

      # TODO
      # speedLimit, speedLimitSign
      instruction.allManeuvers = maneuvers

    self.pm.send('navInstructionNda', msg)



  class RouteTCPServer(socketserver.TCPServer):
    def __init__(self, server_address, RequestHandlerClass, navi_route):
      self.navi_route = navi_route
      socketserver.TCPServer.allow_reuse_address = True
      super().__init__(server_address, RequestHandlerClass)      

  class RouteTCPHandler(socketserver.BaseRequestHandler):
    def recv(self, length):
      data = b''
      while len(data) < length:
        chunk = self.request.recv(length - len(data))
        if not chunk:
          break
        data += chunk
      return data

    def setup(self):
      if not Params().get_bool("UseExternalNaviRoutes"):
        Params().put_bool("UseExternalNaviRoutes", True)
      #print("######### ExternalNaviRoute Connected")

    def finish(self):
      #Params().put_bool("UseExternalNaviRoutes", False)
      #print("######### ExternalNaviRoute Closed")
      pass

    def handle(self):
      try:
        length_bytes = self.recv(4)
        if len(length_bytes) == 4:
          length = struct.unpack(">I", length_bytes)[0]
          if length >= 4:
            if length > 1024 * 1024 * 10:
              raise Exception

            self.server.navi_route.last_client_address = self.client_address[0]

            type_bytes = self.recv(4)
            type = struct.unpack(">I", type_bytes)[0]
            data = self.recv(length - 4)

            if type == 0:  # route
              routes = []
              count = int(len(data) / 8)
              print("navi_route_receive", count)

              if count > 0:
                for i in range(count):
                  offset = i * 8
                  lat = struct.unpack(">f", data[offset:offset + 4])[0]
                  lon = struct.unpack(">f", data[offset + 4:offset + 8])[0]

                  coord = Coordinate.from_mapbox_tuple((lon, lat))
                  routes.append(coord)

                coords = [c.as_dict() for c in routes]
                self.server.navi_route.dispatch_route(coords)
              else:
                self.server.navi_route.dispatch_route(None)

            elif type == 1:  # instruction
              self.server.navi_route.dispatch_instruction(json.loads(data.decode('utf-8')))
      except:
        pass

      self.request.close()

def main():
  rk = Ratekeeper(1.0)
  navi_route = NaviRoute()
  while True:
    navi_route.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
