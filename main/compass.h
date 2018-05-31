
#define QMC5883L_ADDRESS              (0x0D)
#define QMC5883L_REG_OUT_X_L          (0x00)
#define QMC5883L_REG_OUT_X_M          (0x01)
#define QMC5883L_REG_OUT_Y_L          (0x02)
#define QMC5883L_REG_OUT_Y_M          (0x03)
#define QMC5883L_REG_OUT_Z_L          (0x04)
#define QMC5883L_REG_OUT_Z_M          (0x05)
#define QMC5883L_REG_STATUS           (0x06)
#define QMC5883L_REG_OUT_TEMP_L       (0x07)
#define QMC5883L_REG_OUT_TEMP_M       (0x08)
#define QMC5883L_REG_CONFIG_1         (0x09)
#define QMC5883L_REG_CONFIG_2         (0x0A)
#define QMC5883L_REG_SET_RESET        (0x0B)
#define QMC5883L_REG_ID               (0x0D)


#define QMC5883L_VAL_SR               (0x01)
#define QMC5883L_VAL_CONF_1           (0x01)
#define QMC5883L_VAL_CONF_2           (0x40)

void init_compass();
void check_compass_config();
void read_compass_values(int16_t*, int16_t*, int16_t*);
void read_temperature(int16_t*);
