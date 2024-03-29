# origin https://github.com/popedison/LSM9DS1

class XM:
    ADDRESS = 0x6B
    WHO_AM_I = 0x0F    # r
    WHO_AM_I_OK = 0x68  #r
    ACT_THS = 0x04 #rw
    ACT_DUR = 0x05 # rw
    INT_GEN_CFG_XL = 0x06 # rw
    INT_GEN_THS_X_XL = 0x07 # rw
    INT_GEN_THS_Y_XL = 0x08 # rw
    INT_GEN_THS_Z_XL = 0x09 # rw
    INT_GEN_DUR_XL = 0x0A # rw
    REFERENCE_G = 0x0B # rw
    INT1_CTRL = 0x0C # rw
    INT2_CTRL = 0x0D # rw
    CTRL_REG1_G = 0x10 #rw
    CTRL_REG2_G = 0x11 #rw
    CTRL_REG3_G = 0x12 # rw
    ORIENT_CFG_G = 0x13 # rw
    INT_GEN_SRC_G = 0x14 # r
    OUT_TEMP_L = 0x15 # r
    OUT_TEMP_H = 0x16 # r
    STATUS_REG = 0x17 # r
    OUT_X_L_G = 0x18 # r
    OUT_X_H_G = 0x19 # r
    OUT_Y_L_G = 0x1A # r
    OUT_Y_H_G = 0x1B # r
    OUT_Z_L_G = 0x1C # r
    OUT_Z_H_G = 0x1D # r
    CTRL_REG4 = 0x1E # rw
    CTRL_REG5_XL = 0x1F # rw
    CTRL_REG6_XL = 0x20 # rw
    CTRL_REG7_XL = 0x21 # rw
    CTRL_REG8 = 0x22 # rw
    CTRL_REG9 = 0x23 # rw
    CTRL_REG10 = 0x24 # rw
    Reserved = 0x25 # none
    INT_GEN_SRC_XL = 0x26 # r
    STATUS_REG = 0x27 # r
    OUT_X_L_XL = 0x28 # r
    OUT_X_H_XL = 0x29 # r
    OUT_Y_L_XL = 0x2A # r
    OUT_Y_H_XL = 0x2B # r
    OUT_Z_L_XL = 0x2C # r
    OUT_Z_H_XL = 0x2D # r
    FIFO_CTRL = 0x2E # rw
    FIFO_SRC = 0x2F # r
    INT_GEN_CFG_G = 0x30 # rw
    INT_GEN_THS_XH_G = 0x31 # rw
    INT_GEN_THS_XL_G = 0x32 # rw
    INT_GEN_THS_YH_G = 0x33 # rw
    INT_GEN_THS_YL_G = 0x34 # rw
    INT_GEN_THS_ZH_G = 0x35 # rw
    INT_GEN_THS_ZL_G = 0x36 # rw
    INT_GEN_DUR_G =  0x37 # rw

    RANGE_A = {'2G':(0b00 << 3),'4G':(0b10 << 3),'8G':(0b11 << 3),'16G':(0b01 << 3)}
    CAL_A = {'2G': (2.0/32768.0),'4G': (4.0/32768.0),'8G':(8.0/32768.0),'16G':(16.0/32768.0)}
    
    CAL_TEMP = 1.0/8.0
    
    RANGE_G = {'245DPS': (0b00 << 3), '500DPS': (0b01 << 3), '2000DPS': (0b11 << 3)}
    CAL_G = {'245DPS': (245.0/32768.0), '500DPS': (500.0/32768.0), '2000DPS': (2000.0/32768.0)}



# Class for magenetic compas configuration and registers
class MAG:
    ADDRESS = 0x1E
    WHO_AM_I = 0x0F # r
    WHO_AM_I_OK = 0x3D #r
    
    OFFSET_X_REG_L_M = 0x05 # rw
    OFFSET_X_REG_H_M = 0x06 # rw
    OFFSET_Y_REG_L_M = 0x07 # rw
    OFFSET_Y_REG_H_M = 0x08 # rw
    OFFSET_Z_REG_L_M = 0x09 # rw
    OFFSET_Z_REG_H_M = 0x0A # rw

    CTRL_REG1_M = 0x20 # rw
    CTRL_REG2_M = 0x21 # rw
    CTRL_REG3_M = 0x22 # rw
    CTRL_REG4_M = 0x23 # rw
    CTRL_REG5_M = 0x24 # rw
    
    STATUS_REG_M = 0x27 # r
    OUT_X_L_M = 0x28 # r
    OUT_X_H_M = 0x29 # r
    OUT_Y_L_M = 0x2A # r
    OUT_Y_H_M = 0x2B # r
    OUT_Z_L_M = 0x2C # r
    OUT_Z_H_M = 0x2D # r

    INT_CFG_M = 0x30 # rw
    INT_SRC_M = 0x31 # r
    INT_TSH_L_M = 0x32 # rw
    INT_TSH_H_M = 0x33 # rw

    RANGE_M = {'2GAUSS': (0b00 << 5), '4GAUSS': (0b01 << 5), '8GAUSS': (0b10 << 5), '12GAUSS': (0b11 << 5)}
    CAL_M = {'2GAUSS': (2.0/32768.0), '4GAUSS': (4.0/32768.0), '8GAUSS': (8.0/32768.0), '12GAUSS': (12.0/32768.0)}

