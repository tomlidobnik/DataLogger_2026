# [FreeRTOS](https://www.freertos.org/)

---

FreeRTOS je realnočasni operacijski sistem za vgrajene sisteme, ki omogoča multitasking na mikrokontrolerjih z omejenimi viri. V projektu DataLogger_2026 se uporablja za vzporedno izvajanje nalog, kot so beleženje podatkov na SD kartico, prenos podatkov preko NRF modula in spremljanje sistema, kar izboljša odzivnost in učinkovitost v primerjavi z while(1) zanko.

FreeRTOS podpira več ključnih konceptov:

-   Naloge (Tasks) - neodvisne enote izvajanja kode
-   Čakalne vrste (Queues) - za komunikacijo med nalogami
-   Semaphore - za sinhronizacijo
-   Timerji - za periodične operacije

---

### Prednosti in slabosti

| Prednosti                                                  | Slabosti                                                                |
| ---------------------------------------------------------- | ----------------------------------------------------------------------- |
| + Majhna velikost in nizka poraba virov.                   | - Dodatna kompleksnost pri razvoju.                                     |
| + Realnočasno izvajanje z določljivimi odzivnimi časi.     | - Potrebno poznavanje RTOS konceptov.                                   |
| + Odprtokodni in brezplačen.                               | - Lahko povzroči težave z sinhronizacijo če ni pravilno implementirano. |
| + Široka podpora za različne arhitekture (vključno STM32). | - Dodatna režija pri preklapljanju nalog.                               |
| + Aktivna skupnost in dobra dokumentacija.                 | - Ni primeren za zelo preproste aplikacije.                             |
| + Integracija s STM32CubeMX za enostavno nastavitev.       |                                                                         |

---

### Licenca

-   MIT License

-   [FreeRTOS License](https://www.freertos.org/a00114.html)

---

### Število uporabnikov

-   FreeRTOS ima samo 8 tisoč zvezdic a menim, da se uporablja na več milijon naprav po vsem svetu, vključno z različnimi industrijskimi aplikacijami, IoT napravami in avtomobilsko industrijo.

---

### Časovna in prostorska zahtevnost

-   Časovna zahtevnost: Malo počasnejša kot preprost while(1) zanka zaradi režije pri preklapljanju nalog, vendar omogoča boljšo berljivost kode in vzdrževanje.
-   Prostorska zahtevnost: Minimalna - jedro zahteva ~4-9KB FLASH in ~1-2KB RAM, odvisno od konfiguracije.

---

### Vzdrževanje - št. razvijalcev, zadnja sprememba

-   Aktivno vzdrževan s strani Amazon Web Services (AWS)
-   Velika skupnost razvijalcev in prispevkov
-   Zadnja posodobitev: FreeRTOS v11.1.0 (2024)

---

## Uporaba

### Ključni koncepti v projektu DataLogger_2026

| **Koncept**  | Opis                                                |
| ------------ | --------------------------------------------------- |
| `Tasks`      | Neodvisne naloge za beleženje in NRF prenos         |
| `Queues`     | Za komunikacijo med interrupt handlerji in nalogami |
| `Semaphores` | Za sinhronizacijo dostopa do skupnih virov          |
| `Timers`     | Za periodične operacije (100Hz logging, 10Hz NRF)   |
| `Mutexes`    | Za zaščito kritičnih sekcij kode                    |

---

#### **Arhitektura nalog v projektu**

-   StartLoggingTask: Beleži podatke na SD kartico pri 100Hz
-   StartNRFTask: Prenosi podatke preko NRF modula pri 10Hz
-   Interrupt callbacks: Asinhrono posodabljajo globalne spremenljivke

---
#### 1. **Osnovna struktura nalog v projektu**


-   **loggingTask**: Naloga za beleženje podatkov na SD kartico pri 100 Hz (vsakih 10 ms).
-   **nrfTask**: Naloga za prenos podatkov preko NRF modula pri 10 Hz (vsakih 100 ms).

Te naloge omogočajo, da se izvajajo vzporedno z interrupt handlerji (npr. za ADC, CAN), kar izboljša odzivnost sistema v primerjavi z eno veliko `while(1)` zanko.

#### 2. **Kako ustvariti nalogo**

##### **a. Definicija atributov nalog**

Vsaka naloga ima strukturo `osThreadAttr_t`, ki določa njene lastnosti. Primer za vaše naloge:

```c
// Definicija za loggingTask
osThreadId_t loggingTaskHandle;
const osThreadAttr_t loggingTask_attributes = {
  .name = "loggingTask",           // Ime naloge (za debug)
  .stack_size = 1024 * 4,          // Velikost stack-a (4 KB, več kot default zaradi SD operacij)
  .priority = (osPriority_t) osPriorityNormal,  // Prioriteta (normalna)
};

// Podobno za nrfTask
osThreadId_t nrfTaskHandle;
const osThreadAttr_t nrfTask_attributes = {
  .name = "nrfTask",
  .stack_size = 512 * 4,           // Manjši stack, ker je manj kompleksna
  .priority = (osPriority_t) osPriorityNormal,
};
```

-   **Stack size**: Nastavljen glede na potrebe – loggingTask ima večji stack zaradi dela z datotekami in SD kartico.
-   **Prioriteta**: Vsi na `osPriorityNormal` (lahko spremenite, če želite, da ena naloga prednjači, npr. loggingTask na višjo prioriteto za hitrejše beleženje).

##### **b. Ustvarjanje nalog v main() funkciji**

V `main()` funkciji (po inicializaciji periferij) pokličete `osThreadNew` za vsako nalogo:

```c
/* Init scheduler */
osKernelInitialize();

/* Create the thread(s) */
loggingTaskHandle = osThreadNew(StartLoggingTask, NULL, &loggingTask_attributes);
nrfTaskHandle = osThreadNew(StartNRFTask, NULL, &nrfTask_attributes);

/* Start scheduler */
osKernelStart();
```


##### **c. Implementacija funkcij nalog**

Vsaka naloga je definirana kot funkcija z neskončno zanko `for (;;)`:

-   **StartLoggingTask** (iz vašega primera):

    ```c
    void StartLoggingTask(void *argument) {
        for (;;) {
            if (loggingStarted == 0) {
                loggingRes = initLogging(logFileName, &fs);
                if (loggingRes == FR_OK) {
                    loggingStarted = 1;
                }
            }
            if (logNext == 1) {
                // Beleženje podatkov na SD (vaša logika iz originalne while(1))
                logNext = 0;
            }
            if (secondsCounter > 600) {
                NVIC_SystemReset();  // Reset po 10 minutah
            }
            osDelay(10);  // Zakasnitev za 10 ms (100 Hz)
        }
    }
    ```

    -   Uporablja `osDelay(10)` za periodično izvajanje.

-   **StartNRFTask**:

    ```c
    void StartNRFTask(void *argument) {
        float torque = 1.1;
        float airmFloat = 1.1;
        float airpFloat = 1.1;
        uint8_t pnum = 0;

        for (;;) {
            if (__HAL_TIM_GET_COUNTER(&htim17) >= 25) { // 10Hz
                pnum++;
                tx_completed = 0;
                if (pnum < 4) {
                    // Prepare and send payload
                    NRF_Upload_Payload(rf_payload, 32);
                    CE_Pulse();
                } else if (pnum == 4) {
                    // AIR data payload
                    NRF_Upload_Payload(rf_payload, 32);
                    CE_Pulse();
                } else {
                    pnum = 0;
                    // Battery data payload
                    NRF_Upload_Payload(rf_payload, 32);
                    CE_Pulse();
                }
                TIM17->CNT = 0;
            }
            osDelay(1); // Check frequently for timing
        }
    }
    ```

---

#### Interrupt callback za posodobitev podatkov:

```c
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == hadc2.Instance) {
        waterTemp1.raw = adc_buff1[0];
        waterTemp1.voltage = waterTemp1.raw * vrefCoef;
        // Calculate temperature
        waterTemp1.value = WaterTemp_COEFICIENT * log(tempRes) + WaterTemp_VACANT_MEMBER;
        // ... other ADC processing
    }
}
```
