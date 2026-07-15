#!/usr/bin/env python3
import os


HEADER = """VERSION ""


NS_ :
    NS_DESC_
    CM_
    BA_DEF_
    BA_
    VAL_
    CAT_DEF_
    CAT_
    FILTER
    BA_DEF_DEF_
    EV_DATA_
    ENVVAR_DATA_
    SGTYPE_
    SGTYPE_VAL_
    BA_DEF_SGTYPE_
    BA_SGTYPE_
    SIG_TYPE_REF_
    SIG_GROUP_
    SIG_VALTYPE_
    SIGTYPE_VALTYPE_
    BO_TX_BU_
    BA_DEF_REL_
    BA_REL_
    BA_DEF_DEF_REL_
    BU_SG_REL_
    BU_EV_REL_
    BU_BO_REL_
    SG_MUL_VAL_

BS_:

BU_: XXX RADAR

"""


def write_object_msg(f, addr):
  name = f"CORNER_RADAR_430_OBJECTS_{addr:x}"
  f.write(f"""
BO_ {addr} {name}: 32 RADAR
 SG_ HEADER_BYTE_0 : 0|8@1+ (1,0) [0|255] "" XXX
 SG_ HEADER_BYTE_1 : 8|8@1+ (1,0) [0|255] "" XXX
 SG_ HEADER_BYTE_2 : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ HEADER_BYTE_3 : 24|8@1+ (1,0) [0|255] "" XXX
""")
  for slot in range(7):
    bit = 32 + slot * 32
    prefix = f"SLOT{slot + 1}"
    f.write(f""" SG_ {prefix}_DISTANCE_RAW : {bit}|13@1+ (1,0) [0|8191] "" XXX
 SG_ {prefix}_META_13_15 : {bit + 13}|3@1+ (1,0) [0|7] "" XXX
 SG_ {prefix}_META_BYTE_2 : {bit + 16}|8@1+ (1,0) [0|255] "" XXX
 SG_ {prefix}_META_BYTE_3 : {bit + 24}|8@1+ (1,0) [0|255] "" XXX
""")
  f.write(f"""CM_ BO_ {addr} "IONIQ 9 candidate corner radar bin container. 0x430-0x437 correlate with left side and 0x440-0x447 with right side. Bytes 4-31 appear as seven 4-byte slots; lower 13 bits are a distance candidate at 0.05 m scale. This DBC intentionally exposes only raw candidate fields until lateral and velocity fields are validated.";
BA_ "GenMsgCycleTime" BO_ {addr} 30;
""")


if __name__ == "__main__":
  dbc_name = os.path.basename(__file__).replace(".py", ".dbc")
  out_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), dbc_name)
  with open(out_path, "w", encoding="ascii") as f:
    f.write(HEADER)
    f.write('BA_DEF_ BO_  "GenMsgCycleTime" INT 0 100000;\n')
    f.write('BA_DEF_DEF_  "GenMsgCycleTime" 0;\n')
    for addr in range(0x430, 0x438):
      write_object_msg(f, addr)
    for addr in range(0x440, 0x448):
      write_object_msg(f, addr)
