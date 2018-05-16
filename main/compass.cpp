#include "driver/i2c.h"
#include "driver/gpio.h"
#include "compass.h"

void init_compass()
{
	// Initialize i2c driver
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_21;
	conf.scl_io_num = GPIO_NUM_22;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	i2c_cmd_handle_t cmd;

	// Send the order to clear line and select device
	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (HMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	// Register A config
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (HMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, HMC5883L_REG_CONFIG_A, I2C_MASTER_ACK); // Mode register
	i2c_master_write_byte(cmd, HMC5883L_CONFIG_B, I2C_MASTER_ACK); // value 0
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	// Register B config
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (HMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, HMC5883L_REG_CONFIG_B, I2C_MASTER_ACK); // Mode register
	i2c_master_write_byte(cmd, HMC5883L_CONFIG_B, I2C_MASTER_ACK); // value 0
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	// Register MODE to continuous readings
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (HMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, HMC5883L_REG_MODE, I2C_MASTER_ACK); // Mode register
	i2c_master_write_byte(cmd, HMC5883L_CONFIG_MODE, I2C_MASTER_ACK); // value 0
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

void check_compass_config()
{
	i2c_cmd_handle_t cmd;
	uint8_t data;
	for (int i = 0; i<13; i++ ){
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (HMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, i, I2C_MASTER_ACK)); // Data registers
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (HMC5883L_ADDRESS << 1) | I2C_MASTER_READ, I2C_MASTER_ACK));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &data, I2C_MASTER_ACK));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);

		printf("%d: %X\n", i, data);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

void read_compass_values(int16_t* x, int16_t* y, int16_t* z)
{
	i2c_cmd_handle_t cmd;
	uint8_t data[6];
	for (int i = 3; i<9; i++ ){
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (HMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, i, I2C_MASTER_ACK)); // Data registers
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (HMC5883L_ADDRESS << 1) | I2C_MASTER_READ, I2C_MASTER_ACK));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &data[i-3], I2C_MASTER_ACK));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	}

	x[0] = (uint16_t)(data[0])<<8 | (uint16_t)(data[1]);
	y[0] = (uint16_t)(data[2])<<8 | (uint16_t)(data[3]);
	z[0] = (uint16_t)(data[4])<<8 | (uint16_t)(data[5]);
}