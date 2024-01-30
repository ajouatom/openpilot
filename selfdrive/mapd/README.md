# MapD
The OpenStreetMap based speed logical by the [move-fast team](https://github.com/move-fast)

[opsline](https://github.com/move-fast/opspline) which is the pkg needed to run OSM on EON/C2 by [move-fast team](https://github.com/move-fast).
The c3 uses regular SciPy. We upload gps tracks to improve OSM mapping. Better mapping better osm experience you will have.

## customization
To change the speed offset go to `/selfdrive/controls/lib/speed_limit_controller.py` L18,19. `LIMIT_PERC_OFFSET_V` is % you add your set speed. `_LIMIT_PERC_OFFSET_BP` is the speed in m/s you to detect the change at. For example, you want to go 33mph in speedzone of 30mph zone `LIMIT_PERC_OFFSET_V = [0.1]` adds 10% more to based and `_LIMIT_PERC_OFFSET_BP = [13.4]` is 13.4 m/s(30mph) add those together and you get a set speed of 33mph. You may also change the breaking to make it brake earlier or slower by editing `selfdrive/controls/lib/drive_helpers.py` L23-L28.

Other controls can be customized at `/selfdrive/controls/lib/turn_speed_controller.py` and `/selfdrive/controls/lib/vision_turn_controller.py`.






  © OpenStreetMap contributors
