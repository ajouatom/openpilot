#!/usr/bin/env python3
import os


if __name__ == "__main__":
  dbc_name = os.path.basename(__file__).replace(".py", ".dbc")
  hyundai_path = os.path.dirname(os.path.realpath(__file__))
  with open(os.path.join(hyundai_path, dbc_name), "w", encoding="utf-8") as f:
    f.write("""
VERSION ""


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
    VAL_TABLE_
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

BU_: XXX
    """)

    # Denso DNMWR006 radar used by the 2018-2019 Kia Sorento UM. Raw object
    # tracks are published at 20 Hz in 45 slots. A range value of 0xfff8 is an
    # empty slot; validity is derived in RadarInterface rather than the DBC.
    for address in range(0x500, 0x52D):
      f.write(f"""
BO_ {address} RADAR_TRACK_{address:x}: 8 RADAR
 SG_ COUNTER : 7|8@0+ (1,0) [0|255] "" XXX
 SG_ LONG_DIST : 15|16@0+ (0.00625,0) [0|409.59375] "m" XXX
 SG_ AZIMUTH : 31|11@0- (0.25,0) [-256|255.75] "deg" XXX
 SG_ OBJECT_FLAGS_1 : 36|5@0+ (1,0) [0|31] "" XXX
 SG_ OBJECT_FLAG_2 : 47|1@0+ (1,0) [0|1] "" XXX
 SG_ REL_SPEED : 46|12@0- (0.015625,0) [-32|31.984375] "m/s" XXX
 SG_ OBJECT_FLAGS_3 : 50|11@0+ (1,0) [0|2047] "" XXX
      """)
