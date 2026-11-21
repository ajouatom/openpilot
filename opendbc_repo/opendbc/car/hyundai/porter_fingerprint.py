"""Conservative passive identification for the owner's classic Porter EV harness.

Used only by the opt-in two-vehicle switch. This is not a VIN or an identity for
all Porter variants. Unseen traffic or CAN-FD prevents this fallback.
"""

# Complete bus-0 application-ID/DLC set observed in the owner's Porter logs.
PORTER_BUS0 = {
  127: 8, 304: 8, 320: 8, 339: 8, 352: 8, 356: 4, 544: 8, 593: 8,
  688: 5, 881: 8, 882: 8, 897: 8, 902: 8, 903: 8, 916: 8,
  1040: 8, 1042: 8, 1098: 4, 1102: 8, 1136: 8, 1151: 8, 1168: 8,
  1173: 8, 1225: 8, 1265: 4, 1280: 1, 1287: 4, 1291: 8, 1292: 8,
  1294: 8, 1322: 8, 1345: 8, 1355: 8, 1363: 8, 1369: 8, 1379: 8,
  1394: 8, 1407: 8, 1414: 4, 1426: 8, 1429: 8, 1430: 8, 1456: 4,
  1473: 8, 1507: 8, 1535: 8,
}
PORTER_CAMERA = {832: 8, 909: 8, 1069: 8, 1155: 8, 1342: 6}


def matches_porter(fingerprint):
  # Diagnostic responses/queries are not vehicle fingerprints. Some startup
  # safety modes also forward camera traffic back onto the vehicle bus.
  bus0 = {a: n for a, n in fingerprint.get(0, {}).items() if a < 0x700}
  allowed = PORTER_BUS0 | PORTER_CAMERA
  return (all(n <= 8 for bus, frames in fingerprint.items() if bus < 128 for n in frames.values()) and
          all(bus0.get(a) == n for a, n in PORTER_BUS0.items()) and
          all(allowed.get(a) == n for a, n in bus0.items()) and
          fingerprint.get(2, {}).get(832) == 8)
