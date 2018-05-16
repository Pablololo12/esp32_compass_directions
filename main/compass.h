

#define HMC5883L_ADDRESS              (0x1E)
#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)
#define HMC5883L_CONFIG_A             (0x74)
#define HMC5883L_CONFIG_B             (0x20)
#define HMC5883L_CONFIG_MODE          (0x00)

void init_compass();
void check_compass_config();
void read_compass_values(int16_t*, int16_t*, int16_t*);
