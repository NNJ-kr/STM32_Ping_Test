/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/tcp.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* DoIP (ISO 13400-2) */
#define DOIP_HDR_LEN            8U
#define DOIP_RX_BUF_SIZE        2048U        /* 최대 메시지 = 8 + 4 + 2 + 1024 = 1038B */
#define DOIP_PT_GENERIC_NACK    0x0000U
#define DOIP_PT_ROUTING_REQ     0x0005U
#define DOIP_PT_ROUTING_RES     0x0006U
#define DOIP_PT_DIAG_MSG        0x8001U
#define DOIP_PT_DIAG_ACK        0x8002U
#define DOIP_PT_DIAG_NACK       0x8003U
#define DOIP_ECU_ADDR           0x1000U      /* 이 보드(ECU)의 논리 주소 */

/* OTA 수신 영역: Sector 5 (256KB) */
#define OTA_BANK_START          0x08040000U
#define OTA_BANK_END            0x08080000U  /* Sector 5 끝 (이 주소는 포함 안 함) */
#define OTA_BANK_SECTOR         FLASH_SECTOR_5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint32_t current_flash_addr = OTA_BANK_START;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================================
 *  DoIP (ISO 13400-2) 수신 계층 + UDS (ISO 14229-1) 0x36 / 0x37 처리
 *
 *  TCP는 메시지 경계가 없는 스트림이라 수신 콜백 1번 = 메시지 1개가 아니다.
 *  그래서 받은 바이트를 doip_rx_buf에 쌓아두고,
 *  DoIP 헤더(8B)의 Payload Length만큼 다 모였을 때만 메시지 1개로 처리한다.
 * ==========================================================================*/

/* ---------- DoIP 세션 상태 ---------- */
static uint8_t  doip_rx_buf[DOIP_RX_BUF_SIZE];
static uint16_t doip_rx_len    = 0;      /* 버퍼에 쌓인 바이트 수 */
static uint16_t tester_addr    = 0;      /* Routing Activation 때 받은 테스터 주소 */
static uint8_t  routing_active = 0;

/* ---------- OTA(UDS 0x36/0x37) 상태 ---------- */
static uint8_t  ota_in_progress = 0;
static uint8_t  expected_bsc    = 1;

/* ===================== DoIP 송신 ===================== */
static void doip_send(struct tcp_pcb *tpcb, uint16_t ptype,
                      const uint8_t *payload, uint16_t plen)
{
    uint8_t hdr[DOIP_HDR_LEN] = {
        0x02, 0xFD,                                   /* Protocol Version / Inverse */
        (uint8_t)(ptype >> 8), (uint8_t)ptype,        /* Payload Type */
        0x00, 0x00, (uint8_t)(plen >> 8), (uint8_t)plen /* Payload Length */
    };

    tcp_write(tpcb, hdr, DOIP_HDR_LEN, TCP_WRITE_FLAG_COPY | TCP_WRITE_FLAG_MORE);
    if (plen > 0) {
        tcp_write(tpcb, payload, plen, TCP_WRITE_FLAG_COPY);
    }
    tcp_output(tpcb);
}

/* 헤더 오류 응답 (Payload Type 0x0000) */
static void doip_send_generic_nack(struct tcp_pcb *tpcb, uint8_t code)
{
    doip_send(tpcb, DOIP_PT_GENERIC_NACK, &code, 1);
}

/* Routing Activation Response (0x0006) */
static void doip_send_routing_response(struct tcp_pcb *tpcb, uint8_t code)
{
    uint8_t pl[9] = {
        (uint8_t)(tester_addr >> 8), (uint8_t)tester_addr,       /* 테스터 주소 */
        (uint8_t)(DOIP_ECU_ADDR >> 8), (uint8_t)DOIP_ECU_ADDR,   /* ECU 주소 */
        code,                                                    /* 0x10 = 성공 */
        0x00, 0x00, 0x00, 0x00                                   /* Reserved */
    };
    doip_send(tpcb, DOIP_PT_ROUTING_RES, pl, sizeof(pl));
}

/* Diagnostic Message ACK(0x8002) / NACK(0x8003)
 * 요청의 SA/TA를 뒤집어서 보낸다 (보내는 쪽 = ECU, 받는 쪽 = 테스터) */
static void doip_send_diag_status(struct tcp_pcb *tpcb, uint16_t ptype,
                                  uint16_t req_sa, uint16_t req_ta, uint8_t code)
{
    uint8_t pl[5] = {
        (uint8_t)(req_ta >> 8), (uint8_t)req_ta,
        (uint8_t)(req_sa >> 8), (uint8_t)req_sa,
        code
    };
    doip_send(tpcb, ptype, pl, sizeof(pl));
}

/* UDS 응답을 Diagnostic Message(0x8001)로 감싸서 보낸다 (짧은 응답 전용) */
static void uds_send_response(struct tcp_pcb *tpcb, const uint8_t *uds, uint16_t len)
{
    uint8_t pl[4 + 8];
    if (len > 8) {
        return;
    }
    pl[0] = (uint8_t)(DOIP_ECU_ADDR >> 8);
    pl[1] = (uint8_t)DOIP_ECU_ADDR;
    pl[2] = (uint8_t)(tester_addr >> 8);
    pl[3] = (uint8_t)tester_addr;
    memcpy(&pl[4], uds, len);
    doip_send(tpcb, DOIP_PT_DIAG_MSG, pl, (uint16_t)(4 + len));
}

static void uds_send_nrc(struct tcp_pcb *tpcb, uint8_t sid, uint8_t nrc)
{
    uint8_t res[3] = { 0x7F, sid, nrc };
    uds_send_response(tpcb, res, sizeof(res));
}

/* ===================== Flash / OTA ===================== */
/* OTA 상태를 처음으로 되돌린다. 실패, 재시도, 연결 끊김 때마다 호출 */
static void ota_reset(void)
{
    ota_in_progress    = 0;
    expected_bsc       = 1;
    current_flash_addr = OTA_BANK_START;
    HAL_FLASH_Lock();
}

static HAL_StatusTypeDef ota_erase_bank(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_ALL_ERRORS);

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = OTA_BANK_SECTOR;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    return HAL_FLASHEx_Erase(&erase, &sector_error);
}

/* ===================== UDS 서비스 ===================== */
/* 0x36 TransferData: [0x36][BSC][data...] */
static void uds_transfer_data(struct tcp_pcb *tpcb, const uint8_t *uds, uint16_t len)
{
    if (len < 2) {
        uds_send_nrc(tpcb, 0x36, 0x13);           /* incorrectMessageLengthOrInvalidFormat */
        return;
    }

    uint8_t        bsc      = uds[1];
    const uint8_t *data     = &uds[2];
    uint16_t       data_len = (uint16_t)(len - 2);

    if (bsc != expected_bsc) {
        ota_reset();
        uds_send_nrc(tpcb, 0x36, 0x73);           /* wrongBlockSequenceCounter */
        return;
    }
    if ((data_len % 4U) != 0U) {
        ota_reset();
        uds_send_nrc(tpcb, 0x36, 0x13);           /* 워드 정렬 안 됨 */
        return;
    }

    /* 첫 블록이면 새 OTA 시작: 쓰기 주소 초기화 + 뱅크 erase */
    if (!ota_in_progress) {
        current_flash_addr = OTA_BANK_START;
        if (ota_erase_bank() != HAL_OK) {
            ota_reset();
            uds_send_nrc(tpcb, 0x36, 0x72);       /* generalProgrammingFailure */
            return;
        }
        ota_in_progress = 1;
    }

    /* 뱅크 범위를 넘어가면 중단 */
    if ((current_flash_addr + data_len) > OTA_BANK_END) {
        ota_reset();
        uds_send_nrc(tpcb, 0x36, 0x71);           /* transferDataSuspended */
        return;
    }

    for (uint16_t i = 0; i < data_len; i += 4) {
        uint32_t word = (uint32_t)data[i]
                      | ((uint32_t)data[i + 1] << 8)
                      | ((uint32_t)data[i + 2] << 16)
                      | ((uint32_t)data[i + 3] << 24);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, current_flash_addr, word) != HAL_OK
            || *(volatile uint32_t *)current_flash_addr != word) {   /* 쓴 값 다시 읽어서 확인 */
            ota_reset();
            uds_send_nrc(tpcb, 0x36, 0x72);       /* generalProgrammingFailure */
            return;
        }
        current_flash_addr += 4;
    }

    expected_bsc++;                               /* 0xFF 다음은 0x00 (ISO 14229 규칙) */

    uint8_t res[2] = { 0x76, bsc };
    uds_send_response(tpcb, res, sizeof(res));
}

/* 0x37 RequestTransferExit */
static void uds_transfer_exit(struct tcp_pcb *tpcb)
{
    if (!ota_in_progress) {
        uds_send_nrc(tpcb, 0x37, 0x24);           /* requestSequenceError */
        return;
    }
    ota_reset();                                  /* Flash Lock 포함 */

    uint8_t res[1] = { 0x77 };
    uds_send_response(tpcb, res, sizeof(res));
}

static void uds_handle_request(struct tcp_pcb *tpcb, const uint8_t *uds, uint16_t len)
{
    switch (uds[0]) {
    case 0x36: uds_transfer_data(tpcb, uds, len); break;
    case 0x37: uds_transfer_exit(tpcb);           break;
    default:   uds_send_nrc(tpcb, uds[0], 0x11);  break;   /* serviceNotSupported */
    }
}

/* ===================== DoIP 메시지 처리 ===================== */
static void doip_handle_message(struct tcp_pcb *tpcb, uint16_t ptype,
                                const uint8_t *payload, uint16_t plen)
{
    switch (ptype) {
    case DOIP_PT_ROUTING_REQ:
    {
        if (plen < 7) {                           /* SA(2) + Type(1) + Reserved(4) */
            doip_send_generic_nack(tpcb, 0x04);   /* invalid payload length */
            break;
        }
        tester_addr    = (uint16_t)((payload[0] << 8) | payload[1]);
        routing_active = 1;
        doip_send_routing_response(tpcb, 0x10);   /* Routing successfully activated */
        break;
    }

    case DOIP_PT_DIAG_MSG:
    {
        if (plen < 5) {                           /* SA(2) + TA(2) + UDS 최소 1바이트 */
            doip_send_generic_nack(tpcb, 0x04);
            break;
        }
        uint16_t sa = (uint16_t)((payload[0] << 8) | payload[1]);
        uint16_t ta = (uint16_t)((payload[2] << 8) | payload[3]);

        if (!routing_active || sa != tester_addr) {
            doip_send_diag_status(tpcb, DOIP_PT_DIAG_NACK, sa, ta, 0x02);  /* invalid source address */
            break;
        }
        if (ta != DOIP_ECU_ADDR) {
            doip_send_diag_status(tpcb, DOIP_PT_DIAG_NACK, sa, ta, 0x03);  /* unknown target address */
            break;
        }

        doip_send_diag_status(tpcb, DOIP_PT_DIAG_ACK, sa, ta, 0x00);       /* 먼저 "받았음" */
        uds_handle_request(tpcb, &payload[4], (uint16_t)(plen - 4));       /* 그다음 UDS 응답 */
        break;
    }

    default:
        doip_send_generic_nack(tpcb, 0x01);       /* unknown payload type */
        break;
    }
}

/* 버퍼에 완성된 메시지가 있으면 전부 처리하고, 처리한 만큼 버퍼 앞에서 지운다 */
static void doip_process_buffer(struct tcp_pcb *tpcb)
{
    while (doip_rx_len >= DOIP_HDR_LEN) {
        if (doip_rx_buf[0] != 0x02 || doip_rx_buf[1] != 0xFD) {
            doip_send_generic_nack(tpcb, 0x00);   /* incorrect pattern format */
            doip_rx_len = 0;
            return;
        }

        uint16_t ptype = (uint16_t)((doip_rx_buf[2] << 8) | doip_rx_buf[3]);
        uint32_t plen  = ((uint32_t)doip_rx_buf[4] << 24) | ((uint32_t)doip_rx_buf[5] << 16)
                       | ((uint32_t)doip_rx_buf[6] << 8)  |  (uint32_t)doip_rx_buf[7];

        if (plen > (DOIP_RX_BUF_SIZE - DOIP_HDR_LEN)) {
            doip_send_generic_nack(tpcb, 0x02);   /* message too large */
            doip_rx_len = 0;
            return;
        }

        uint16_t total = (uint16_t)(DOIP_HDR_LEN + plen);
        if (doip_rx_len < total) {
            return;                               /* 아직 덜 왔음 → 다음 조각 기다림 */
        }

        doip_handle_message(tpcb, ptype, &doip_rx_buf[DOIP_HDR_LEN], (uint16_t)plen);

        doip_rx_len = (uint16_t)(doip_rx_len - total);
        memmove(doip_rx_buf, &doip_rx_buf[total], doip_rx_len);
    }
}

/* 연결이 끊기면 세션과 OTA 상태를 모두 초기화 */
static void doip_reset_session(void)
{
    doip_rx_len    = 0;
    routing_active = 0;
    tester_addr    = 0;
    if (ota_in_progress) {
        ota_reset();
    }
}

/* ===================== LwIP 콜백 ===================== */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p == NULL) {                              /* 상대가 연결 종료 (FIN) */
        doip_reset_session();
        tcp_close(tpcb);
        return ERR_OK;
    }
    if (err != ERR_OK) {
        pbuf_free(p);
        return err;
    }

    /* pbuf(체인일 수도 있음)를 버퍼에 이어 붙이면서, 완성된 메시지는 바로 처리 */
    uint16_t copied = 0;
    while (copied < p->tot_len) {
        uint16_t space = (uint16_t)(DOIP_RX_BUF_SIZE - doip_rx_len);
        uint16_t chunk = (uint16_t)(p->tot_len - copied);
        if (chunk > space) {
            chunk = space;
        }
        if (chunk == 0) {                         /* 버퍼가 꽉 찼는데 메시지가 안 끝남 → 비정상 */
            doip_rx_len = 0;
            break;
        }
        pbuf_copy_partial(p, &doip_rx_buf[doip_rx_len], chunk, copied);
        doip_rx_len = (uint16_t)(doip_rx_len + chunk);
        copied      = (uint16_t)(copied + chunk);

        doip_process_buffer(tpcb);
    }

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

/* 연결이 비정상 종료(RST 등)됐을 때. tpcb는 이미 해제된 상태라 건드리지 않는다 */
static void tcp_server_err(void *arg, err_t err)
{
    doip_reset_session();
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    doip_reset_session();                         /* 새 연결은 항상 깨끗한 상태에서 시작 */
    tcp_recv(newpcb, tcp_server_recv);
    tcp_err(newpcb, tcp_server_err);
    return ERR_OK;
}

void tcp_server_init(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (pcb == NULL) {
        return;
    }
    if (tcp_bind(pcb, IP_ADDR_ANY, 13400) != ERR_OK) {
        tcp_close(pcb);
        return;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, tcp_server_accept);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */

  tcp_server_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MX_LWIP_Process();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
