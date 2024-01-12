/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_i2c

#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#include <am_mcu_apollo.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>

LOG_MODULE_REGISTER(ambiq_i2c, CONFIG_I2C_LOG_LEVEL);

typedef int (*ambiq_i2c_pwr_func_t)(void);

#define PWRCTRL_MAX_WAIT_US 5
#define I2C_TRANSFER_TIMEOUT_MSEC 500	/* Transfer timeout period */

#include "i2c-priv.h"

struct i2c_ambiq_config {
	uint32_t base;
	int size;
	uint32_t bitrate;
	const struct pinctrl_dev_config *pcfg;
	ambiq_i2c_pwr_func_t pwr_func;
	void (*irq_config_func)(void);
};

typedef void (*i2c_ambiq_callback_t)(const struct device *dev, int result, void *data);

struct i2c_ambiq_data {
	am_hal_iom_config_t iom_cfg;
	void *IOMHandle;
	struct k_sem bus_sem;
	struct k_sem transfer_sem;
	i2c_ambiq_callback_t callback;
	void *callback_data;
	uint32_t *pDMATCBBuffer;
};

#ifdef CONFIG_I2C_AMBIQ_DMA
static void
pfnI2C_Callback(void *pCallbackCtxt, uint32_t status)
{
	const struct device *dev = pCallbackCtxt;
	struct i2c_ambiq_data *data = dev->data;

	if(data->callback)
	{
		data->callback(dev, status, data->callback_data);
	}
	k_sem_give(&data->transfer_sem);
}

static void i2c_ambiq_isr(const struct device *dev)
{
	uint32_t      ui32Status;
	struct i2c_ambiq_data *data = dev->data;

	am_hal_iom_interrupt_status_get(data->IOMHandle, &ui32Status, false);
	am_hal_iom_interrupt_clear(data->IOMHandle, ui32Status);
	am_hal_iom_interrupt_service(data->IOMHandle, ui32Status);
}
#endif

static int i2c_ambiq_read(const struct device *dev, struct i2c_msg *msg, uint16_t addr)
{
	struct i2c_ambiq_data *data = dev->data;

	int ret = 0;

	am_hal_iom_transfer_t trans = {0};

	trans.ui8Priority = 1;
	trans.eDirection = AM_HAL_IOM_RX;
	trans.uPeerInfo.ui32I2CDevAddr = addr;
	trans.ui32NumBytes = msg->len;
	trans.pui32RxBuffer = (uint32_t *)msg->buf;

#ifdef CONFIG_I2C_AMBIQ_DMA
	ret = am_hal_iom_nonblocking_transfer(data->IOMHandle, &trans, pfnI2C_Callback, (void *)dev);
	if (k_sem_take(&data->transfer_sem, K_MSEC(I2C_TRANSFER_TIMEOUT_MSEC))) {
		LOG_ERR("Timeout waiting for transfer complete");
		return -ETIMEDOUT;
	}
#else
	ret = am_hal_iom_blocking_transfer(data->IOMHandle, &trans);
#endif
	return ret;
}

static int i2c_ambiq_write(const struct device *dev, struct i2c_msg *msg, uint16_t addr)
{
	struct i2c_ambiq_data *data = dev->data;

	int ret = 0;

	am_hal_iom_transfer_t trans = {0};

	trans.ui8Priority = 1;
	trans.eDirection = AM_HAL_IOM_TX;
	trans.uPeerInfo.ui32I2CDevAddr = addr;
	trans.ui32NumBytes = msg->len;
	trans.pui32TxBuffer = (uint32_t *)msg->buf;

#ifdef CONFIG_I2C_AMBIQ_DMA
	ret = am_hal_iom_nonblocking_transfer(data->IOMHandle, &trans, pfnI2C_Callback, (void *)dev);

	if (k_sem_take(&data->transfer_sem, K_MSEC(I2C_TRANSFER_TIMEOUT_MSEC))) {
		LOG_ERR("Timeout waiting for transfer complete");
		return -ETIMEDOUT;
	}
#else
	ret = am_hal_iom_blocking_transfer(data->IOMHandle, &trans);
#endif

	return ret;
}

static int i2c_ambiq_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_ambiq_data *data = dev->data;

	if (!(I2C_MODE_CONTROLLER & dev_config)) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		data->iom_cfg.ui32ClockFreq = AM_HAL_IOM_100KHZ;
		break;
	case I2C_SPEED_FAST:
		data->iom_cfg.ui32ClockFreq = AM_HAL_IOM_400KHZ;
		break;
	case I2C_SPEED_FAST_PLUS:
		data->iom_cfg.ui32ClockFreq = AM_HAL_IOM_1MHZ;
		break;
	default:
		return -EINVAL;
	}

#ifdef CONFIG_I2C_AMBIQ_DMA
	data->iom_cfg.pNBTxnBuf = data->pDMATCBBuffer;
	data->iom_cfg.ui32NBTxnBufLength = CONFIG_I2C_DMA_TCB_BUFFER_SIZE;
#endif

	am_hal_iom_configure(data->IOMHandle, &data->iom_cfg);

	return 0;
}

static int i2c_ambiq_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			      uint16_t addr)
{
	struct i2c_ambiq_data *data = dev->data;
	int ret = 0;

	if (!num_msgs) {
		return 0;
	}

	/* Send out messages */
	k_sem_take(&data->bus_sem, K_FOREVER);

	for (int i = 0; i < num_msgs; i++) {
		if (msgs[i].flags & I2C_MSG_READ) {
			ret = i2c_ambiq_read(dev, &(msgs[i]), addr);
		} else {
			ret = i2c_ambiq_write(dev, &(msgs[i]), addr);
		}

		if (ret != 0) {
			return ret;
		}
	}

	k_sem_give(&data->bus_sem);

	return 0;
}

static int i2c_ambiq_init(const struct device *dev)
{
	struct i2c_ambiq_data *data = dev->data;
	const struct i2c_ambiq_config *config = dev->config;
	uint32_t bitrate_cfg = i2c_map_dt_bitrate(config->bitrate);
	int ret = 0;
	void *buf = NULL;

	data->iom_cfg.eInterfaceMode = AM_HAL_IOM_I2C_MODE;

	if(AM_HAL_STATUS_SUCCESS != am_hal_iom_initialize((config->base - REG_IOM_BASEADDR) / config->size,
					&data->IOMHandle)) {
		LOG_ERR("Fail to initialize I2C\n");
		return -ENXIO;
	}

	config->pwr_func();

#ifdef CONFIG_I2C_AMBIQ_DMA
	buf = k_malloc(CONFIG_I2C_DMA_TCB_BUFFER_SIZE * 4);
	if (buf == NULL) {
		ret = -ENOMEM;
		goto end;
	}
	data->pDMATCBBuffer = (uint32_t*)buf;
#endif

	ret = i2c_ambiq_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (ret < 0) {
		LOG_ERR("Fail to config I2C\n");
		goto end;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Fail to config I2C pins\n");
		goto end;
	}

#ifdef CONFIG_I2C_AMBIQ_DMA
	am_hal_iom_interrupt_clear(data->IOMHandle, AM_HAL_IOM_INT_CQMODE);
	am_hal_iom_interrupt_enable(data->IOMHandle, AM_HAL_IOM_INT_CQMODE);
	config->irq_config_func();
#endif

	if(AM_HAL_STATUS_SUCCESS != am_hal_iom_enable(data->IOMHandle)) {
		LOG_ERR("Fail to enable I2C\n");
		ret = -EIO;
	}
end:
	if (ret < 0) {
		am_hal_iom_uninitialize(data->IOMHandle);
		if(buf != NULL) {
			k_free((void*)data->pDMATCBBuffer);
		}
	}
	return ret;
}

static struct i2c_driver_api i2c_ambiq_driver_api = {
	.configure = i2c_ambiq_configure,
	.transfer = i2c_ambiq_transfer,
};

#define AMBIQ_I2C_DEFINE(n)                                                                        \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static int pwr_on_ambiq_i2c_##n(void)                                                      \
	{                                                                                          \
		uint32_t addr = DT_REG_ADDR(DT_INST_PHANDLE(n, ambiq_pwrcfg)) +                    \
				DT_INST_PHA(n, ambiq_pwrcfg, offset);                              \
		sys_write32((sys_read32(addr) | DT_INST_PHA(n, ambiq_pwrcfg, mask)), addr);        \
		k_busy_wait(PWRCTRL_MAX_WAIT_US);                                                  \
		return 0;                                                                          \
	}                                                                                          \
	static void i2c_irq_config_func_##n(void)                                              \
	{                                                                                       \
		IRQ_CONNECT(DT_INST_IRQN(n),                                                        \
		DT_INST_IRQ(n, priority),                                                   \
		i2c_ambiq_isr,                                                             \
		DEVICE_DT_INST_GET(n), 0);                                                  \
		irq_enable(DT_INST_IRQN(n));                                                        \
	};                                                                                      \
	static struct i2c_ambiq_data i2c_ambiq_data##n;                                            \
	static const struct i2c_ambiq_config i2c_ambiq_config##n = {                               \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.size = DT_INST_REG_SIZE(n),                                                       \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.irq_config_func = i2c_irq_config_func_##n,                                        \
		.pwr_func = pwr_on_ambiq_i2c_##n};                                                 \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_ambiq_init, NULL, &i2c_ambiq_data##n,                     \
				  &i2c_ambiq_config##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,     \
				  &i2c_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_I2C_DEFINE)
