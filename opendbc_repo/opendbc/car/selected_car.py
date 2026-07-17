def get_selected_car_platform(name: str):
  from opendbc.car.ford.values import CAR as FORD
  from opendbc.car.hyundai.values import CAR as HYUNDAI
  from opendbc.car.gm.values import CAR as GM
  from opendbc.car.toyota.values import CAR as TOYOTA
  from opendbc.car.mazda.values import CAR as MAZDA
  from opendbc.car.volkswagen.values import CAR as VOLKSWAGEN
  from opendbc.car.tesla.values import CAR as TESLA

  platforms = [platform for brand in (FORD, GM, TOYOTA, HYUNDAI, MAZDA, VOLKSWAGEN) for platform in brand]
  # Model X is intentionally dashcam-only. Model 3/Y have a CarController and
  # must be selectable even when automatic fingerprinting is unavailable.
  platforms.extend((TESLA.TESLA_MODEL_3, TESLA.TESLA_MODEL_Y))

  return next((platform for platform in platforms for doc in platform.config.car_docs if name == doc.name), None)
