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


def write_status_msg(f):
  f.write("""
BO_ 560 CORNER_RADAR_235_STATUS_230: 16 RADAR
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ ACTIVE_OBJECT_MSG_COUNT : 24|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_4 : 32|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_5 : 40|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_6 : 48|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_7 : 56|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_8 : 64|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_9 : 72|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_10 : 80|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_11 : 88|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_12 : 96|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_13 : 104|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_14 : 112|8@1+ (1,0) [0|255] "" XXX
 SG_ STATUS_BYTE_15 : 120|8@1+ (1,0) [0|255] "" XXX
CM_ BO_ 560 "Candidate corner radar status/header on bus 1. Seen around 33 Hz near object messages 0x235-0x248. ACTIVE_OBJECT_MSG_COUNT matches the number of active 0x235+ object containers in the sampled route.";
BA_ "GenMsgCycleTime" BO_ 560 30;
""")


def write_object_msg(f, addr):
  name = f"CORNER_RADAR_235_OBJECTS_{addr:x}"
  f.write(f"""
BO_ {addr} {name}: 32 RADAR
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ OBJ_QUAL_LEVEL : 24|7@1+ (1,0) [0|100] "%" XXX
 SG_ OBJ_ALIVE_AGE : 32|8@1+ (1,0) [0|255] "" XXX
 SG_ OBJ_MOVING_FLAG : 40|4@1+ (1,0) [0|15] "" XXX
 SG_ OBJ_ID : 44|7@1+ (1,0) [0|127] "" XXX
 SG_ OBJ_WIDTH : 52|8@1+ (0.01,0) [0|2.55] "m" XXX
 SG_ OBJ_CLASS : 60|4@1+ (1,0) [0|15] "" XXX
 SG_ OBJ_REL_POS_X : 64|13@1+ (0.05,0) [0|409.55] "m" XXX
 SG_ OBJ_REL_POS_Y : 78|12@1+ (0.05,-102.4) [-102.4|102.35] "m" XXX
 SG_ OBJ_REL_VEL_X : 91|12@1+ (0.05,-100) [-100|104.75] "m/s" XXX
 SG_ OBJ_REL_VEL_Y : 104|10@1+ (0.05,-25) [-25|26.15] "m/s" XXX
 SG_ OBJ_REL_ACCEL_X : 115|9@1- (0.05,0) [-12.8|12.75] "m/s^2" XXX
 SG_ OBJ_SYNC_FRAME_INDEX : 124|4@1+ (1,0) [0|15] "" XXX
 SG_ OBJ_ABS_VEL_X : 128|12@1+ (0.05,-102.4) [-102.4|102.35] "m/s" XXX
 SG_ TAIL_SIGNAL_140 : 140|12@1+ (1,0) [0|4095] "" XXX
 SG_ TAIL_SIGNAL_152 : 152|7@1+ (1,0) [0|127] "" XXX
 SG_ TAIL_SIGNAL_160 : 160|8@1+ (1,0) [0|255] "" XXX
 SG_ TAIL_SIGNAL_168 : 168|4@1+ (1,0) [0|15] "" XXX
 SG_ TAIL_SIGNAL_172 : 172|7@1+ (1,0) [0|127] "" XXX
 SG_ TAIL_SIGNAL_180 : 180|7@1+ (1,0) [0|127] "" XXX
 SG_ TAIL_SIGNAL_188 : 188|3@1+ (1,0) [0|7] "" XXX
""")

  f.write(f"""CM_ BO_ {addr} "Candidate new corner radar object payload on bus 1. 0x235-0x248 run near 33 Hz. Signal split and scales are derived from the public ZENDAR CAN-FD DBC FR_CMR object layout, then checked against the sampled IONIQ 5 PE rlog. In the sample, 0x235-0x239 carry active objects and 0x23a-0x248 mostly carry default payloads.";
CM_ SG_ {addr} OBJ_REL_POS_X "ZENDAR FR_CMR_Obj_RelPosX-compatible candidate. In the sampled route, active slots decode to plausible object longitudinal positions.";
CM_ SG_ {addr} OBJ_REL_POS_Y "ZENDAR FR_CMR_Obj_RelPosY-compatible candidate. Negative values were seen for objects on the right side in the sampled route.";
CM_ SG_ {addr} OBJ_REL_ACCEL_X "This matches the GitHub hit at 115|9@1- named FR_CMR_Obj_RelAccelXxxVal in the public ZENDAR evaluation DBC.";
CM_ SG_ {addr} TAIL_SIGNAL_140 "Unconfirmed tail field. Public ZENDAR FR_CMR layout uses this region for the next object's absolute velocity/metadata, but bytes 24-31 were zero in the sampled IONIQ 5 PE route.";
BA_ "GenMsgCycleTime" BO_ {addr} 30;
""")


if __name__ == "__main__":
  dbc_name = os.path.basename(__file__).replace(".py", ".dbc")
  out_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), dbc_name)
  with open(out_path, "w", encoding="ascii") as f:
    f.write(HEADER)
    f.write('BA_DEF_ BO_  "GenMsgCycleTime" INT 0 100000;\n')
    f.write('BA_DEF_DEF_  "GenMsgCycleTime" 0;\n')
    write_status_msg(f)
    for addr in range(0x235, 0x249):
      write_object_msg(f, addr)
