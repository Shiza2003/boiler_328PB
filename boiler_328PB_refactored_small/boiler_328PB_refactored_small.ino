/*
  =====================================================================
  ПАНЕЛЬ УПРАВЛЕНИЯ БОЙЛЕРОМ — Arduino (ATmega328P / ATmega328PB)
  Целевая конфигурация: ATmega328PB, 16 MHz; сборка Arduino IDE
  совместимым профилем Arduino Uno / ATmega328P (стандартный загрузчик).
  Двуцветный OLED 128x64 (I2C): верхние 16 строк — жёлтые, нижние 48 —
  белые. DS3231 (I2C), 6 кнопок, выход сигнала 5 В.

  Библиотеки — Arduino IDE → Скетч → Подключить библиотеку →
  Управлять библиотеками... → ввести в поиск:
    1) «U8g2»    — автор olikraus (дисплей + кириллица)
    2) «RTClib»  — автор Adafruit (только класс DateTime; обмен с DS3231 — свой, программный)
       IDE предложит «Install all» — согласитесь: это поставит
       зависимость Adafruit BusIO.
  Wire и EEPROM входят в состав Arduino.

  Раскладка экрана:
    • жёлтая зона 0..15 px — ТОЛЬКО заголовки и номер стола;
    • белая зона 16..63 px — 4 пункта по 12 px;
    • стол 1: часы 30 px (строки 19..48), таймер 12 px (строки 52..63).

  Столы (▲/▼ — листание, без анимации):
    1. Часы + обратный отсчёт и огонёк в правом нижнем углу.
    2. Настройки: Время / Таймер (длительность) / День недели.
    3. Таймеры по дням: карусель дней → список → Старт и Длит.
  Кнопка ⏱ (PIN_TIMER): на столе 1 — запуск/отмена ручного таймера,
  на столах 2 и 3 — «Назад».

  Надёжность ввода (исправления):
    • кнопки опрашиваются по ТАЙМЕРНОМУ ПРЕРЫВАНИЮ каждые 2 мс и
      фильтруются классическим интегратором (счётчик 0..8: «нажата» при
      достижении 8, «отпущена» при 0) — устойчив и к дребезгу, и к
      наводкам; скорость перерисовки экрана на кнопки не влияет;
    • «залипшая» кнопка (замкнута > 20 с) выключается из опроса,
      остальные работают; при отпускании кнопка снова в строю;
    • ДИАГНОСТИКА КНОПОК: включите питание, удерживая ⏱ — откроется
      экран с живым состоянием всех 6 входов (см. ниже). Выход — удержать
      ⏱ 2 с, дальше обычная загрузка;
    • запись в EEPROM идёт по одному байту за проход цикла и не
      «замораживает» интерфейс;
    • таймеры, пропущенные из-за задержки цикла, срабатывают;
    • связь с RTC и OLED проверяется раз в 0,5 с; если любое из устройств
      не отвечает дольше 5 с — EEPROM дописывается и проверяется, затем
      импульс на A2 (соединён с RESET);
    • логи в USB-UART (USE_LOG, 115200 бод) для поиска зависаний;
    • данные из EEPROM проверяются при загрузке.

  Оптимизация под 2 КБ ОЗУ / 32 КБ flash:
    • страничный буфер дисплея (_1_) — 128 байт вместо 1 КБ;
    • строки и картинки во flash (PROGMEM), без float и snprintf;
    • всего три шрифта U8g2; цифры таймера и дни недели в карусели
      нарисованы своими битовыми картинками (точный размер в пикселях).
  =====================================================================
*/



#include <Wire.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <RTClib.h>

#if defined(F_CPU) && (F_CPU != 16000000UL)
  #error "Прошивка рассчитана на частоту 16 MHz"
#endif

// =====================================================================
//                         НАСТРОЙКА ПИНОВ
// =====================================================================
// Кнопки подключаются между пином и GND (включается INPUT_PULLUP).
const uint8_t PIN_UP    = 2;   // ▲
const uint8_t PIN_DOWN  = 4;   // ▼
const uint8_t PIN_LEFT  = 5;   // ◀
const uint8_t PIN_RIGHT = 3;   // ▶
const uint8_t PIN_OK    = 6;   // ● центральная
const uint8_t PIN_TIMER = 7;   // ⏱ отдельная кнопка (таймер / назад)

// Выход сигнала: +5 В, пока идёт отсчёт таймера (ручного или по
// расписанию) — та же логика, что у индикатора событий в эмуляторе.
const uint8_t PIN_OUT         = 9;
const bool    OUT_ACTIVE_HIGH = true;   // false — инвертировать (LOW при работе)

// Две шины I2C:
//   • аппаратная (Wire): OLED SSD1306 — SDA=A4, SCL=A5 (быстрый кадр);
//   • программная: DS3231 — SDA=A0, SCL=A1 (никогда не блокирует loop()).
// =====================================================================

// ---------------------------- параметры ------------------------------
#define MAX_T            8        // таймеров на день (не больше 9)
#define SAMPLE_MS        2        // период фильтрации кнопок (мс; Timer2 ISR = 1 ms)
#define DEBOUNCE_MAX     8        // порог интегратора (8 × 2 мс = 16 мс)
#define STUCK_MS         20000UL  // кнопка замкнута дольше — считается залипшей
#define REPEAT_DELAY_MS  500      // задержка до автоповтора ▲/▼ в редакторах
#define REPEAT_PERIOD_MS 150
#define SAVE_DELAY_MS    2000     // запись в EEPROM через 2 с после последнего изменения
#define CFG_MAGIC        0xB5
#define DEFAULT_TIMER_S  40

// ----- сброс по потере I2C ------------------------------------------
// A2 соединён с выводом RESET. В покое — вход (Hi-Z), при сбросе — LOW.
const uint8_t PIN_RESET = A2;
// Программная шина (DS3231): без Wire, на любых пинах
const uint8_t SW_SDA = A0;
const uint8_t SW_SCL = A1;
#define RTC_ADDR        0x68      // DS3231 (программная шина A0/A1)
#define OLED_ADDR       0x3C      // OLED (аппаратная шина A4/A5); 0x3D, если распаяно так
#define I2C_CHECK_MS    500       // период проверки связи
#define I2C_LOST_MS     5000UL    // связь не вернулась за это время → сброс
#define RESET_PULSE_MS  50        // длительность импульса на RESET

#if !defined(WIRE_HAS_TIMEOUT)
  #warning "Ядро без Wire.setWireTimeout: зависшая шина OLED заблокирует loop(), сброс не сработает"
#endif

// ----- логи в USB-UART (Serial, пины 0/1) ----------------------------
#define USE_LOG          0        // 0 — отключить диагностику Serial и освободить Flash; включить для отладки
#define LOG_BAUD         115200
#define LOG_SLOW_MS      150      // проход loop() дольше этого (мс) — строка SLOW

#if MAX_T > 9
  #error "MAX_T должно быть не больше 9"
#endif

// Дисплей на аппаратной шине. Для SH1106 (1.3") замените на U8G2_SH1106_128X64_NONAME_1_HW_I2C
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Шрифты (если какой-то не находится — замените на похожий из списка U8g2)
#define F_TXT    u8g2_font_7x13_t_cyrillic    // заголовки, пункты, время в списке
#define F_CLOCK  u8g2_font_logisoso30_tn      // часы, цифры 30 px
#define F_EDIT   u8g2_font_logisoso18_tn      // редактор времени

// Границы зон двуцветного экрана
#define YELLOW_H   16             // высота жёлтой зоны
#define ROW_H      12             // высота пункта в белой зоне

// ---------------------------- типы -----------------------------------
// Все типы объявлены до первой функции — иначе автогенерация прототипов
// в Arduino IDE ломается.
struct Timer { uint32_t s; uint32_t d; };      // старт и длительность, секунды

struct Config {
  uint8_t  magic;
  uint32_t timerSec;          // длительность ручного таймера / по умолчанию
  uint8_t  dowShift;          // сдвиг дня недели относительно календаря
  uint8_t  cnt[7];            // число таймеров по дням (0 = Пн ... 6 = Вс)
  Timer    t[7][MAX_T];
};

enum Key : uint8_t { K_NONE, K_UP, K_DOWN, K_LEFT, K_RIGHT, K_OK, K_BACK };
enum EdKind : uint8_t { ED_CLOCK, ED_TIMERSEC, ED_START, ED_DUR };

struct CarAn { bool on; int8_t dir; uint32_t t0; };

// ---------------------------- строки (flash) -------------------------
const char DSU[7][5] PROGMEM = {"ПН","ВТ","СР","ЧТ","ПТ","СБ","ВС"};
const char DSL[7][5] PROGMEM = {"Пн","Вт","Ср","Чт","Пт","Сб","Вс"};

const char H_SETTINGS[] PROGMEM = "НАСТРОЙКИ";
const char H_TIMERS[]   PROGMEM = "ТАЙМЕРЫ";
const char H_TIMER[]    PROGMEM = "ТАЙМЕР";
const char H_DOW[]      PROGMEM = "ДЕНЬ НЕДЕЛИ";
const char H_TIME[]     PROGMEM = "ВРЕМЯ";
const char H_START[]    PROGMEM = "СТАРТ";
const char H_DUR[]      PROGMEM = "ДЛИТЕЛЬНОСТЬ";
const char L_TIME[]     PROGMEM = "Время";
const char L_TIMER[]    PROGMEM = "Таймер";
const char L_DOW[]      PROGMEM = "День недели";
const char L_START[]    PROGMEM = "Старт";
const char L_DUR[]      PROGMEM = "Длит.";
const char E_NORTC[]    PROGMEM = "Нет часов DS3231";
const char H_DIAG[]     PROGMEM = "ТЕСТ КНОПОК";
const char BN[6][5]     PROGMEM = {"Вв","Вн","Лв","Пр","Ок","Тм"};   // имена кнопок на экране теста

// огонёк 7x8 (на экране масштабируется до 10x12)
const uint8_t FLAME[8] PROGMEM = {0x08, 0x18, 0x1C, 0x3E, 0x6F, 0x7F, 0x7F, 0x3E};

// цифры 5x6 (старший бит слева): таймер рисуется с масштабом 2 → 10x12 px
const uint8_t DIG[10][6] PROGMEM = {
  {0x0E,0x11,0x11,0x11,0x11,0x0E},   // 0
  {0x04,0x0C,0x04,0x04,0x04,0x0E},   // 1
  {0x0E,0x11,0x02,0x04,0x08,0x1F},   // 2
  {0x1E,0x01,0x0E,0x01,0x01,0x1E},   // 3
  {0x02,0x06,0x0A,0x12,0x1F,0x02},   // 4
  {0x1F,0x10,0x1E,0x01,0x01,0x1E},   // 5
  {0x0E,0x10,0x1E,0x11,0x11,0x0E},   // 6
  {0x1F,0x01,0x02,0x04,0x08,0x08},   // 7
  {0x0E,0x11,0x0E,0x11,0x11,0x0E},   // 8
  {0x0E,0x11,0x11,0x0F,0x01,0x0E},   // 9
};

// буквы для крупных дней недели, 5x7, старший бит слева
//                      П  Н  В  Т  С  Р  Ч  Б
const uint8_t GL[8][7] PROGMEM = {
  {0x1F,0x11,0x11,0x11,0x11,0x11,0x11},   // П
  {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},   // Н
  {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},   // В
  {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},   // Т
  {0x0F,0x10,0x10,0x10,0x10,0x10,0x0F},   // С
  {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},   // Р
  {0x11,0x11,0x11,0x0F,0x01,0x01,0x01},   // Ч
  {0x1F,0x10,0x10,0x1E,0x11,0x11,0x1E},   // Б
};
//                           Пн    Вт    Ср    Чт    Пт    Сб    Вс
const uint8_t DAYG[7][2] PROGMEM = {{0,1},{2,3},{4,5},{6,3},{0,3},{4,7},{2,4}};

// ---------------------------- глобальные -----------------------------
Config   cfg;
bool     dirty = false;
uint32_t dirtyAt = 0;

uint32_t nowSec = 0;          // секунд с начала суток
uint8_t  calDow = 0;          // день недели по календарю: 0 = Пн ... 6 = Вс
uint32_t lastRtc = 0;

bool     tActive = false;     // идёт отсчёт таймера
uint32_t tEnd = 0;
uint32_t tDoneUntil = 0;

// ---- кнопки: фильтр дребезга + очередь событий -----------------------
#define EV_Q 16                                  // размер очереди (степень двойки)
const uint8_t BTN_PIN[6] = {PIN_UP, PIN_DOWN, PIN_LEFT, PIN_RIGHT, PIN_OK, PIN_TIMER};
const Key     BTN_KEY[6] = {K_UP,   K_DOWN,   K_LEFT,   K_RIGHT,   K_OK,   K_BACK};

volatile uint8_t evK[EV_Q];        // очередь номеров нажатых кнопок (ISR → loop)
volatile uint8_t evH = 0;          // чтение (loop)
volatile uint8_t evT = 0;          // запись (ISR)
volatile uint8_t downMask = 0;     // биты кнопок, удерживаемых после фильтра
volatile uint8_t stuckMask = 0;    // биты «залипших» кнопок (исключены из опроса)
uint32_t nextRep[6] = {0, 0, 0, 0, 0, 0};   // время следующего автоповтора

// состояние интерфейса
uint8_t desk = 0;                 // текущий стол 0..2
bool    setActive = false;        // стол 2: «выбран» (после ●)
uint8_t setSel = 0;               // стол 2: пункт 0..2
uint8_t setSub = 0;               // 0 список, 1 время, 2 таймер, 3 день недели
uint8_t tLevel = 0;               // стол 3: 0 карусель, 1 список, 2 меню таймера, 3 редактор
uint8_t tSel = 255;               // стол 3: выбранный день (255 — ещё не задан)
uint8_t dsSel = 0, dsTop = 0;     // список таймеров дня
bool    dsDel = false;            // выбран столбец «удалить»
uint8_t tmSel = 0, tmIdx = 0;     // меню таймера: пункт, индекс таймера
EdKind  edKind = ED_CLOCK;
uint8_t edF = 0;                  // разряд: 0 часы, 1 минуты, 2 секунды
uint32_t edBlink = 0;             // мигание на границе

CarAn dowAn = {false, 0, 0};      // карусель в настройке дня недели
CarAn tAn   = {false, 0, 0};      // карусель на столе 3

// ------------------------- связь по I2C ------------------------------
uint8_t  i2cLost   = 0;        // маска пропавших устройств: 1 = RTC, 2 = OLED (0 — всё в порядке)
uint32_t i2cLostAt = 0;        // когда связь пропала, мс

// =====================================================================
//        ЛОГИ В USB-UART (Serial, 115200) — ловля зависаний
// =====================================================================
//  BOOT/PREV  при старте: причина прошлого перезапуска
//  HB hh:mm:ss o<выход> n<проходов/с> L<макс проход, мс> R<макс RTC, мс>
//     D<макс отрисовка, мс> f<мин. свободное ОЗУ, байт> i<маска потери I2C>
//     x<пропущено строк>
//  KEY / TMR / SCHED / EE / I2C / RTC / SLOW / STUCK — события
// Лог не тормозит программу: если буфер UART занят, строка пропускается (x).
#if USE_LOG
#define DBG_MAGIC 0xB011E521UL

struct DbgRec {                      // в .noinit: переживает сброс по RESET
  uint32_t magic;
  uint32_t upMs;                     // последний известный uptime, мс
  uint16_t rstCnt;                   // сколько раз был сброс по потере I2C
  int16_t  minFree;                  // минимум свободного ОЗУ, байт
  uint8_t  reason;                   // 0 = работал, 1 = сброс по потере I2C
  uint8_t  lostMask;                 // что пропало: 1 = RTC, 2 = OLED
};
DbgRec  dbg __attribute__((section(".noinit")));
uint8_t mcusrCopy __attribute__((section(".noinit")));

// До main(): запоминаем флаги причины сброса (1=PWR 2=EXT 4=BOD 8=WDT)
void earlyInit(void) __attribute__((naked, used, section(".init3")));
void earlyInit(void) { mcusrCopy = MCUSR; MCUSR = 0; }

uint16_t logDrop = 0, rtcBad = 0, eeWr = 0;
uint16_t loopCnt = 0, mxLoop = 0, mxRtc = 0, mxDraw = 0;
uint32_t lastLoopMs = 0, lastHbMs = 0;
uint8_t  lastStuck = 0;
const char LOG_KN[] PROGMEM = "UPDNLFRTOKTM";      // имена кнопок по 2 символа

extern int __heap_start, *__brkval;
int freeRam() { int v; return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval); }

#define MEM_SAMPLE()   do { int f_ = freeRam(); if (f_ < dbg.minFree) dbg.minFree = f_; } while (0)
#define T_BEGIN(v)     uint32_t v = millis()
#define T_MAX(m, v)    do { uint32_t d_ = millis() - (v); if (d_ > (m)) (m) = (d_ > 65535UL) ? 65535U : (uint16_t)d_; } while (0)
#define LOGMSG(s)      logMsg(F(s))
#define LOGVAL(s, v)   logVal(F(s), (uint32_t)(v))
#define LOGS(s)        Serial.println(F(s))        // с ожиданием: для setup и перед сбросом
#define LOG_FLUSH()    Serial.flush()
#define CNT_INC(v)     ((v)++)
#define LOG_KEY(i)     logKey(i)
#define LOG_LOOP()     logLoop()
#define LOG_EE_DONE()  logEeDone()
#define LOG_JUMP(ns)   logRtcJump(ns)
#define DBG_I2C_RESET(m) do { dbg.reason = 1; dbg.lostMask = (m); dbg.rstCnt++; dbg.upMs = millis(); } while (0)
#define DBG_CLEAR()    (dbg.reason = 0)

// Место в буфере есть? Иначе строка пропускается.
bool logBegin(uint8_t len) {
  if (Serial.availableForWrite() < len + 15) { logDrop++; return false; }
  Serial.print('['); Serial.print(millis()); Serial.print(F("] "));
  return true;
}
void logMsg(const __FlashStringHelper* s) {
  if (!logBegin((uint8_t)strlen_P((PGM_P)s))) return;
  Serial.println(s);
}
void logVal(const __FlashStringHelper* s, uint32_t v) {
  if (!logBegin((uint8_t)strlen_P((PGM_P)s) + 10)) return;
  Serial.print(s); Serial.println(v);
}
void p2(uint8_t v) { if (v < 10) Serial.print('0'); Serial.print(v); }
void logHMS(uint32_t s) { p2(s / 3600); Serial.print(':'); p2((s / 60) % 60); Serial.print(':'); p2(s % 60); }

void logKey(uint8_t i) {                              // вызывается ДО обработки нажатия
  if (!logBegin(30)) return;
  Serial.print(F("KEY "));
  Serial.print((char)pgm_read_byte(&LOG_KN[i * 2]));
  Serial.print((char)pgm_read_byte(&LOG_KN[i * 2 + 1]));
  Serial.print(F(" desk=")); Serial.print(desk);
  Serial.print(F(" sub=")); Serial.print(setSub);
  Serial.print(F(" lvl=")); Serial.println(tLevel);
}
void logRtcJump(uint32_t ns) {                        // время прыгнуло не на 0..2 с
  static bool had = false;
  uint32_t dd = (ns + 86400UL - nowSec) % 86400UL;
  if (had && dd > 2 && logBegin(27)) {
    Serial.print(F("RTC JUMP ")); logHMS(nowSec); Serial.print(F("->")); logHMS(ns); Serial.println();
  }
  had = true;
}
void logEeDone() { logVal(F("EE saved bytes="), eeWr); eeWr = 0; }

void logLoop() {
  uint32_t now = millis();
  if (lastLoopMs) {
    uint32_t dt = now - lastLoopMs;
    if (dt > mxLoop) mxLoop = (dt > 65535UL) ? 65535U : (uint16_t)dt;
    if (dt >= LOG_SLOW_MS) logVal(F("SLOW loop ms="), dt);
  }
  lastLoopMs = now;
  loopCnt++;
  if (stuckMask != lastStuck) { lastStuck = stuckMask; logVal(F("STUCK mask="), lastStuck); }
  if (now - lastHbMs >= 1000UL) {
    lastHbMs = now;
    if (logBegin(48)) {
      Serial.print(F("HB ")); logHMS(nowSec);
      Serial.print(F(" o")); Serial.print(tActive ? 1 : 0);
      Serial.print(F(" n")); Serial.print(loopCnt);
      Serial.print(F(" L")); Serial.print(mxLoop);
      Serial.print(F(" R")); Serial.print(mxRtc);
      Serial.print(F(" D")); Serial.print(mxDraw);
      Serial.print(F(" f")); Serial.print(dbg.minFree);
      Serial.print(F(" i")); Serial.print(i2cLost);
      Serial.print(F(" x")); Serial.println(logDrop);
    }
    loopCnt = 0; mxLoop = 0; mxRtc = 0; mxDraw = 0;
    dbg.upMs = now;
  }
}

void logBoot() {
  Serial.println();
  Serial.print(F("===== BOOT ")); Serial.print(F(__DATE__)); Serial.print(' '); Serial.println(F(__TIME__));
  Serial.print(F("MCUSR=0x")); Serial.println(mcusrCopy, HEX);
  if (dbg.magic != DBG_MAGIC) {
    Serial.println(F("PREV: cold start"));
    dbg.rstCnt = 0;
  } else if (dbg.reason == 1) {
    Serial.print(F("PREV: *** RESET BY I2C LOST: "));
    if (dbg.lostMask & 1) Serial.print(F("RTC "));
    if (dbg.lostMask & 2) Serial.print(F("OLED"));
    Serial.println(F(" ***"));
    Serial.print(F("  up=")); Serial.print(dbg.upMs);
    Serial.print(F("ms resets=")); Serial.println(dbg.rstCnt);
  } else {
    Serial.println(F("PREV: *** UNEXPECTED RESET (power/EMI/crash?) ***"));
    Serial.print(F("  last HB up=")); Serial.print(dbg.upMs); Serial.println(F("ms"));
  }
  Serial.print(F("cfg=")); Serial.print((uint16_t)sizeof(cfg));
  Serial.print(F("B freeRAM=")); Serial.print(freeRam());
#if defined(WIRE_HAS_TIMEOUT)
  Serial.println(F(" WireTimeout=ON"));
#else
  Serial.println(F(" WireTimeout=OFF"));
#endif
  dbg.magic = DBG_MAGIC; dbg.reason = 0; dbg.lostMask = 0; dbg.minFree = 32767;
}
#else
  #define MEM_SAMPLE()
  #define T_BEGIN(v)
  #define T_MAX(m, v)
  #define LOGMSG(s)
  #define LOGVAL(s, v)
  #define LOGS(s)
  #define LOG_FLUSH()
  #define CNT_INC(v)
  #define LOG_KEY(i)
  #define LOG_LOOP()
  #define LOG_EE_DONE()
  #define LOG_JUMP(ns)
  #define DBG_I2C_RESET(m)
  #define DBG_CLEAR()
#endif

// ------------------------------ время --------------------------------
uint8_t dowNow() { return (calDow + cfg.dowShift) % 7; }

// ------------- программный I2C (A0/A1) и драйвер DS3231 ---------------
// Открытый сток: LOW = выход 0, «1» = вход с подтяжкой. Растяжка такта
// ждётся не дольше ~0,1 мс, поэтому зависнуть здесь нельзя.
static inline void swLow(uint8_t p)  { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
static inline void swHigh(uint8_t p) { pinMode(p, INPUT_PULLUP); }

bool swSclUp() {
  swHigh(SW_SCL);
  for (uint8_t i = 0; i < 50; i++) { if (digitalRead(SW_SCL)) return true; delayMicroseconds(2); }
  return false;                                        // SCL удерживается слейвом
}
bool swStart() {
  swHigh(SW_SDA);
  if (!swSclUp()) return false;
  delayMicroseconds(5);
  if (digitalRead(SW_SDA) == LOW) return false;        // SDA залипла
  swLow(SW_SDA); delayMicroseconds(5);
  swLow(SW_SCL); delayMicroseconds(5);
  return true;
}
void swStop() {
  swLow(SW_SDA); delayMicroseconds(5);
  swSclUp(); delayMicroseconds(5);
  swHigh(SW_SDA); delayMicroseconds(5);
}
bool swWrite(uint8_t b) {                              // true = ACK
  for (uint8_t i = 0; i < 8; i++) {
    if (b & 0x80) swHigh(SW_SDA); else swLow(SW_SDA);
    b <<= 1;
    delayMicroseconds(5);
    if (!swSclUp()) return false;
    delayMicroseconds(5);
    swLow(SW_SCL);
  }
  swHigh(SW_SDA); delayMicroseconds(5);                // 9-й такт: ACK
  if (!swSclUp()) return false;
  delayMicroseconds(5);
  bool ack = (digitalRead(SW_SDA) == LOW);
  swLow(SW_SCL); delayMicroseconds(5);
  return ack;
}
uint8_t swRead(bool ack) {
  uint8_t b = 0;
  swHigh(SW_SDA);
  for (uint8_t i = 0; i < 8; i++) {
    delayMicroseconds(5);
    swSclUp();
    delayMicroseconds(5);
    b = (uint8_t)((b << 1) | (digitalRead(SW_SDA) ? 1 : 0));
    swLow(SW_SCL);
  }
  if (ack) swLow(SW_SDA); else swHigh(SW_SDA);
  delayMicroseconds(5); swSclUp(); delayMicroseconds(5);
  swLow(SW_SCL); swHigh(SW_SDA);
  return b;
}
void swRelease() { swHigh(SW_SCL); swHigh(SW_SDA); }   // после ошибки — освободить линии

bool swAck(uint8_t addr) {                             // отвечает ли устройство
  bool ok = swStart() && swWrite((uint8_t)(addr << 1));
  swStop(); swRelease();
  return ok;
}
bool dsRead(uint8_t reg, uint8_t* buf, uint8_t n) {
  bool ok = swStart() && swWrite((uint8_t)(RTC_ADDR << 1)) && swWrite(reg)
         && swStart() && swWrite((uint8_t)((RTC_ADDR << 1) | 1));
  if (ok) for (uint8_t i = 0; i < n; i++) buf[i] = swRead(i + 1 < n);
  swStop(); swRelease();
  return ok;
}
bool dsWrite(uint8_t reg, const uint8_t* buf, uint8_t n) {
  bool ok = swStart() && swWrite((uint8_t)(RTC_ADDR << 1)) && swWrite(reg);
  for (uint8_t i = 0; ok && i < n; i++) ok = swWrite(buf[i]);
  swStop(); swRelease();
  return ok;
}
uint8_t dsB2D(uint8_t v) { return (uint8_t)((v & 0x0F) + 10 * (v >> 4)); }   // BCD → число
uint8_t dsD2B(uint8_t v) { return (uint8_t)(v + 6 * (v / 10)); }              // число → BCD

void dsClearOsf() {                                    // сбросить флаг «питание терялось»
  uint8_t v;
  if (dsRead(0x0F, &v, 1)) { v &= 0x7F; dsWrite(0x0F, &v, 1); }
}
bool dsLostPower() { uint8_t v; return dsRead(0x0F, &v, 1) && (v & 0x80); }
bool dsAdjust(const DateTime& d) {
  uint8_t dw = d.dayOfTheWeek(); if (dw == 0) dw = 7;
  uint8_t b[7] = { dsD2B(d.second()), dsD2B(d.minute()), dsD2B(d.hour()), dw,
                   dsD2B(d.day()), dsD2B(d.month()), dsD2B((uint8_t)(d.year() - 2000)) };
  bool ok = dsWrite(0x00, b, 7);
  if (ok) dsClearOsf();
  return ok;
}

void readRtc() {
  uint8_t b[7];
  if (!dsRead(0x00, b, 7)) { CNT_INC(rtcBad); return; }   // нет ответа — оставляем прежнее время
  uint8_t sc = dsB2D(b[0] & 0x7F), mi = dsB2D(b[1] & 0x7F), h;
  if (b[2] & 0x40) {                                      // 12-часовой режим (на всякий случай)
    h = dsB2D(b[2] & 0x1F);
    if (b[2] & 0x20) { if (h < 12) h += 12; }
    else if (h == 12) h = 0;
  } else h = dsB2D(b[2] & 0x3F);
  if (h > 23 || mi > 59 || sc > 59) { CNT_INC(rtcBad); LOGMSG("RTC garbage read"); return; }   // мусор с шины
  uint32_t ns = (uint32_t)h * 3600UL + (uint32_t)mi * 60UL + sc;
  LOG_JUMP(ns);
  nowSec = ns;
  uint8_t dd = dsB2D(b[4] & 0x3F), mo = dsB2D(b[5] & 0x1F), yy = dsB2D(b[6]);
  if (dd >= 1 && dd <= 31 && mo >= 1 && mo <= 12)
    calDow = (DateTime(2000 + yy, mo, dd, 0, 0, 0).dayOfTheWeek() + 6) % 7;   // RTClib: 0 = Вс
}

void setClockSec(uint32_t t) {                         // меняем только ч:м:с, дата остаётся
  uint8_t b[3] = { dsD2B(t % 60), dsD2B((t / 60) % 60), dsD2B(t / 3600) };
  if (dsWrite(0x00, b, 3)) dsClearOsf();
  readRtc();
}

// ---------------------------- EEPROM ---------------------------------
uint16_t savePos = 0;
void markDirty() { if (!dirty) LOGMSG("EE dirty"); dirty = true; dirtyAt = millis(); savePos = 0; }

void sanitizeCfg() {                       // защита от «мусора» в EEPROM
  if (cfg.timerSec < 1 || cfg.timerSec > 359999UL) cfg.timerSec = DEFAULT_TIMER_S;
  if (cfg.dowShift > 6) cfg.dowShift = 0;
  for (uint8_t d = 0; d < 7; d++) {
    if (cfg.cnt[d] > MAX_T) cfg.cnt[d] = 0;
    for (uint8_t i = 0; i < cfg.cnt[d]; i++) {
      if (cfg.t[d][i].s > 86399UL) cfg.t[d][i].s = 0;
      if (cfg.t[d][i].d < 1 || cfg.t[d][i].d > 359999UL) cfg.t[d][i].d = DEFAULT_TIMER_S;
    }
  }
}

void loadCfg() {
  EEPROM.get(0, cfg);
  if (cfg.magic != CFG_MAGIC) {
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = CFG_MAGIC;
    cfg.timerSec = DEFAULT_TIMER_S;
    markDirty();                            // запишется постепенно, без пауз
  }
  sanitizeCfg();
}

// Запись по одному байту за вызов (~3,4 мс): интерфейс не замирает.
void saveStep() {
  if (!dirty) return;
  if ((millis() - dirtyAt) < SAVE_DELAY_MS) return;
  const uint8_t* p = (const uint8_t*)&cfg;
  for (uint8_t n = 0; n < 32 && savePos < sizeof(cfg); n++, savePos++) {
    if (EEPROM.read(savePos) != p[savePos]) {
      EEPROM.update(savePos, p[savePos]);
      CNT_INC(eeWr);
      savePos++;
      return;
    }
  }
  if (savePos >= sizeof(cfg)) { savePos = 0; dirty = false; LOG_EE_DONE(); }
}

// ----------------------- выход и ручной таймер -----------------------
void outWrite(bool on) {
  digitalWrite(PIN_OUT, (on == OUT_ACTIVE_HIGH) ? HIGH : LOW);
}

void startTimer(uint32_t dur) {
  tActive = true;
  LOGVAL("TMR START s=", dur);
  tEnd = millis() + dur * 1000UL;
  tDoneUntil = 0;
}

void timerToggle() {
  if (tActive) { tActive = false; LOGMSG("TMR STOP (manual)"); }
  else startTimer(cfg.timerSec);
}

// Срабатывание по расписанию. Проверяем все секунды, прошедшие с прошлого
// вызова (до 10 с): таймер не пропадёт, даже если цикл на мгновение завис.
void schedTick() {
  static uint32_t last = 0xFFFFFFFFUL;
  if (last == 0xFFFFFFFFUL) { last = nowSec; return; }
  if (nowSec == last) return;
  uint32_t span = (nowSec + 86400UL - last) % 86400UL;     // сколько секунд прошло
  uint32_t from = last;
  last = nowSec;
  if (span > 10 || tActive) return;                         // время переставили вручную / уже идёт отсчёт
  uint8_t d = dowNow();
  for (uint32_t k = 1; k <= span; k++) {
    uint32_t sec = (from + k) % 86400UL;
    for (uint8_t i = 0; i < cfg.cnt[d]; i++)
      if (cfg.t[d][i].s == sec) { LOGVAL("SCHED fire #", i); startTimer(cfg.t[d][i].d); return; }
  }
}

// ------------------- сброс по потере I2C (RTC / OLED) ----------------
// Устройство отвечает, если принимает свой адрес (ACK).
bool i2cAck(uint8_t addr) {
  Wire.beginTransmission(addr);
  uint8_t e = Wire.endTransmission();
#if defined(WIRE_HAS_TIMEOUT)
  if (Wire.getWireTimeoutFlag()) { Wire.clearWireTimeoutFlag(); return false; }
#endif
  return e == 0;
}

// Дописать в EEPROM только отличающиеся байты и перепроверить (до 4 проходов).
// Проход без отличий = данные записаны и прочитаны верно.
//   0 — EEPROM цел, 1 — копия в ОЗУ повреждена (EEPROM не трогали: в нём
//   последняя хорошая копия), 2 — запись не подтвердилась.
uint8_t eeSyncVerify() {
  if (cfg.magic != CFG_MAGIC) return 1;
  sanitizeCfg();
  const uint8_t* p = (const uint8_t*)&cfg;
  for (uint8_t pass = 0; pass < 4; pass++) {
    bool same = true;
    for (uint16_t i = 0; i < sizeof(cfg); i++)
      if (EEPROM.read(i) != p[i]) { EEPROM.update(i, p[i]); same = false; }
    if (same) { dirty = false; return 0; }
  }
  return 2;
}

void resetByI2C() {
  uint8_t r = eeSyncVerify();
  if (r == 2) {                                        // данные не подтвердились — не сбрасываемся
    LOGS("RESET postponed: EEPROM verify FAILED");
    i2cLostAt = millis();                              // повтор через I2C_LOST_MS
    return;
  }
  if (r == 1) LOGS("EE: RAM copy damaged, EEPROM untouched");
  DBG_I2C_RESET(i2cLost);
  LOGS("RESET: I2C lost, EEPROM ok -> pulse on A2");
  LOG_FLUSH();
  digitalWrite(PIN_RESET, LOW);                        // сначала уровень, потом выход — без выброса
  pinMode(PIN_RESET, OUTPUT);
  delay(RESET_PULSE_MS);
  pinMode(PIN_RESET, INPUT);                           // сюда дойдём, только если RESET не сработал
  DBG_CLEAR();
  LOGS("RESET pulse had no effect");
  i2cLostAt = millis();
}

void i2cWatch() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < I2C_CHECK_MS) return;
  last = now;
  uint8_t m = 0;
  if (!swAck(RTC_ADDR))  m |= 1;
  if (!i2cAck(OLED_ADDR)) m |= 2;
  if (!m) {
    if (i2cLost) { LOGVAL("I2C restored after ms=", now - i2cLostAt); i2cLost = 0; }
    return;
  }
  if (!i2cLost) { i2cLostAt = now; LOGVAL("I2C LOST mask=", m); }
  i2cLost = m;
  if (now - i2cLostAt >= I2C_LOST_MS) resetByI2C();
}

// ------------------------------ кнопки -------------------------------
// Классический фильтр-ИНТЕГРАТОР (Kuhn): у каждой кнопки счётчик 0..8.
// Каждые 2 мс: кнопка замкнута → счётчик +1, разомкнута → −1.
//   • «нажата»   — счётчик дошёл до 8 (≈16 мс устойчивого замыкания);
//   • «отпущена» — счётчик дошёл до 0.
// Дребезг и наводки колеблют счётчик вокруг среднего и до границ не
// доходят. Опрос идёт по таймерному прерыванию (ATmega328P: Timer0
// COMPA, 1 мс) и не зависит от скорости перерисовки экрана.
// Защита от залипания: кнопка, удерживаемая дольше STUCK_MS, исключается
// из опроса до физического отпускания — остальные кнопки не страдают.
void buttonsTick() {                           // вызывается раз в 1 мс
  static uint8_t div = 0;
  static uint8_t integ[6] = {0, 0, 0, 0, 0, 0};
  static uint16_t held[6] = {0, 0, 0, 0, 0, 0};
  if (++div < SAMPLE_MS) return;
  div = 0;
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t bit = (uint8_t)(1 << i);
    if (digitalRead(BTN_PIN[i]) == LOW) { if (integ[i] < DEBOUNCE_MAX) integ[i]++; }
    else if (integ[i] > 0) integ[i]--;
    if (integ[i] == 0) stuckMask &= (uint8_t)~bit;           // физически отпущена
    if (!(downMask & bit)) {
      held[i] = 0;
      if (integ[i] == DEBOUNCE_MAX && !(stuckMask & bit)) {  // нажатие
        downMask |= bit;
        uint8_t nx = (uint8_t)((evT + 1) & (EV_Q - 1));
        if (nx != evH) { evK[evT] = i; evT = nx; }            // событие в очередь
      }
    } else {
      if (integ[i] == 0) downMask &= (uint8_t)~bit;           // отпускание
      else if (++held[i] >= (uint16_t)(STUCK_MS / SAMPLE_MS)) {   // залипла
        downMask &= (uint8_t)~bit;
        stuckMask |= bit;
        held[i] = 0;
      }
    }
  }
}

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)
  #define BTN_ISR 1
  // Timer2 в CTC: 16 MHz / 64 / (249 + 1) = 1 kHz.
  // Timer0 не трогаем: он обслуживает millis()/micros() и PWM.
  // Timer2 не используется этой прошивкой; D3 настроен как вход кнопки.
  ISR(TIMER2_COMPA_vect) { buttonsTick(); }
#else
  #define BTN_ISR 0
  // другие платы: опрос из основного цикла (достаточно часто)
  void buttonsTickLoop() {
    static uint32_t last = 0;
    uint32_t now = millis();
    uint8_t n = 0;
    while ((uint32_t)(now - last) >= 1 && n < 8) { last++; buttonsTick(); n++; }
    if (n == 8) last = now;
  }
#endif

bool editingKey(Key k) {
  if (k != K_UP && k != K_DOWN) return false;
  return (desk == 1 && setActive && setSub != 0) || (desk == 2 && tLevel >= 2);
}

// ----------------------- анимация каруселей дней ---------------------
// Фиксированная точка: 256 = 1.0 (float на AVR дорогой).
uint16_t easeQ(uint16_t t) {                    // вход 0..256 → выход 0..256
  if (t < 128) return (uint16_t)(((uint32_t)2 * t * t) >> 8);
  uint32_t u = 512UL - 2UL * t;                 // 0..256
  return (uint16_t)(256UL - ((u * u) >> 9));
}

uint16_t progressQ(uint32_t t0, uint16_t durMs) {
  uint32_t dt = millis() - t0;
  return dt >= durMs ? 256 : (uint16_t)((dt << 8) / durMs);
}

int16_t carOffsetQ(const CarAn &a) {            // сдвиг карусели в 1/256 ячейки
  if (!a.on) return 0;
  uint16_t p = progressQ(a.t0, 220);
  if (p >= 256) return 0;
  return (int16_t)(a.dir * (int16_t)(256 - easeQ(p)));
}

void goDesk(uint8_t to) { desk = to; }          // листание столов — без анимации

// --------------------------- редакторы -------------------------------
uint32_t edGet() {
  switch (edKind) {
    case ED_CLOCK:    return nowSec;
    case ED_TIMERSEC: return cfg.timerSec;
    case ED_START:    return cfg.t[tSel][tmIdx].s;
    default:          return cfg.t[tSel][tmIdx].d;
  }
}
uint32_t edMin() { return (edKind == ED_TIMERSEC || edKind == ED_DUR) ? 1UL : 0UL; }
uint32_t edMax() { return (edKind == ED_TIMERSEC || edKind == ED_DUR) ? 359999UL : 86399UL; }

void edSet(uint32_t v) {
  switch (edKind) {
    case ED_CLOCK:    setClockSec(v); break;
    case ED_TIMERSEC: cfg.timerSec = v; markDirty(); break;
    case ED_START:    cfg.t[tSel][tmIdx].s = v; markDirty(); break;
    default:          cfg.t[tSel][tmIdx].d = v; markDirty(); break;
  }
}

void edAdj(uint8_t f, int32_t dl) {
  static const int32_t unit[3] = {3600, 60, 1};
  int32_t v = (int32_t)edGet() + dl * unit[f];
  if (v < (int32_t)edMin()) v = (int32_t)edMin();
  if (v > (int32_t)edMax()) v = (int32_t)edMax();
  edSet((uint32_t)v);
}

uint8_t sortDay(uint8_t day, uint8_t idx) {
  uint8_t c = cfg.cnt[day];
  Timer cur = cfg.t[day][idx];
  for (uint8_t i = 1; i < c; i++) {
    Timer x = cfg.t[day][i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && cfg.t[day][j].s > x.s) { cfg.t[day][j + 1] = cfg.t[day][j]; j--; }
    cfg.t[day][j + 1] = x;
  }
  for (uint8_t i = 0; i < c; i++)
    if (cfg.t[day][i].s == cur.s && cfg.t[day][i].d == cur.d) return i;
  return idx;
}

// число строк списка дня: таймеры + строка «+» (если ещё есть место)
uint8_t dayRows(uint8_t c) { return c < MAX_T ? c + 1 : c; }

void clampDs() {                                  // в окне 4 строки
  uint8_t rows = dayRows(cfg.cnt[tSel]);
  if (rows && dsSel >= rows) dsSel = rows - 1;
  if (dsSel < dsTop) dsTop = dsSel;
  if (dsSel > dsTop + 3) dsTop = dsSel - 3;
}

void leaveStartEditor() {
  uint8_t ni = sortDay(tSel, tmIdx);
  dsSel = ni; tmIdx = ni; clampDs();
  markDirty();                                  // новый порядок тоже сохраняем
}

// --------------------------- обработка кнопок ------------------------
void edKey(Key k, int n) {
  if (k == K_LEFT)  edF = (edF + 2) % 3;
  if (k == K_RIGHT) edF = (edF + 1) % 3;
  if (k == K_UP || k == K_DOWN) {
    edAdj(edF, k == K_UP ? n : -n);
    uint32_t t = edGet();
    if (t <= edMin() || t >= edMax()) edBlink = millis() + 800;     // граница — мигаем
  }
}

void settingsKey(Key k, int n) {
  if (setSub == 3) {                                  // день недели (карусель)
    int8_t d = (k == K_UP || k == K_RIGHT) ? 1 : (k == K_DOWN || k == K_LEFT) ? -1 : 0;
    if (d) {
      uint8_t v = (dowNow() + d + 7) % 7;
      cfg.dowShift = (uint8_t)((((int8_t)v - (int8_t)calDow) % 7 + 7) % 7);
      dowAn.on = true; dowAn.dir = d; dowAn.t0 = millis();
      markDirty();
    }
    if (k == K_OK) setSub = 0;
    return;
  }
  if (setSub != 0) {                                  // редактор времени / таймера
    edKey(k, n);
    if (k == K_OK) setSub = 0;
    return;
  }
  if (k == K_UP   && setSel > 0) setSel--;
  if (k == K_DOWN && setSel < 2) setSel++;
  if (k == K_LEFT) setActive = false;
  if (k == K_OK) {
    setSub = setSel + 1;
    edF = 0; edBlink = 0;
    edKind = (setSel == 0) ? ED_CLOCK : ED_TIMERSEC;
  }
}

void dayKey(Key k) {
  uint8_t c = cfg.cnt[tSel];
  if (k == K_UP   && dsSel > 0) dsSel--;
  if (k == K_DOWN && (uint8_t)(dsSel + 1) < dayRows(c)) dsSel++;
  if (k == K_RIGHT && dsSel < c) dsDel = true;
  if (k == K_LEFT) { if (dsDel) dsDel = false; else tLevel = 0; }
  if (k == K_OK) {
    if (dsSel == c) {                                 // строка «+» (есть только если c < MAX_T)
      cfg.t[tSel][c].s = 43200UL;
      cfg.t[tSel][c].d = cfg.timerSec;
      cfg.cnt[tSel] = c + 1;
      dsSel = c; tmIdx = c; tmSel = 0; tLevel = 2;
      markDirty();
    } else if (dsDel) {                               // удалить
      for (uint8_t i = dsSel; i + 1 < c; i++) cfg.t[tSel][i] = cfg.t[tSel][i + 1];
      cfg.cnt[tSel] = c - 1;
      markDirty();
    } else {                                          // открыть меню таймера
      tmIdx = dsSel; tmSel = 0; tLevel = 2;
    }
  }
  clampDs();
}

void tdeskKey(Key k, int n) {
  if (tLevel == 0) {                                  // карусель дней
    int8_t d = (k == K_RIGHT) ? 1 : (k == K_LEFT) ? -1 : 0;
    if (d) { tSel = (tSel + d + 7) % 7; tAn.on = true; tAn.dir = d; tAn.t0 = millis(); }
    if (k == K_OK) { tLevel = 1; dsSel = 0; dsTop = 0; dsDel = false; }
  } else if (tLevel == 1) {
    dayKey(k);
  } else if (tLevel == 2) {                           // меню таймера
    if (k == K_UP) tmSel = 0;
    if (k == K_DOWN) tmSel = 1;
    if (k == K_OK) { edKind = tmSel ? ED_DUR : ED_START; edF = 0; edBlink = 0; tLevel = 3; }
  } else {                                            // редактор Старт / Длит.
    edKey(k, n);
    if (k == K_OK) {
      if (edKind == ED_START) leaveStartEditor();
      tLevel = 2;
    }
  }
}

void uiBack() {
  if (desk == 1) {
    if (setSub != 0) setSub = 0;
    else if (setActive) setActive = false;
    else goDesk(0);
  } else {
    if (tLevel > 0) {
      if (tLevel == 3 && edKind == ED_START) leaveStartEditor();
      tLevel--;
    } else goDesk(0);
  }
}

void onKey(Key k, int n) {
  if (k == K_BACK) {
    if (desk == 1 || desk == 2) uiBack(); else timerToggle();
    return;
  }
  if (desk == 1 && setActive) { settingsKey(k, n); return; }
  if (desk == 2 && (tLevel > 0 || k == K_LEFT || k == K_RIGHT || k == K_OK)) { tdeskKey(k, n); return; }
  if (k == K_UP)   { goDesk((desk + 2) % 3); if (desk == 1) setSel = 2; }
  if (k == K_DOWN) { goDesk((desk + 1) % 3); if (desk == 1) setSel = 0; }
  if (k == K_OK && desk == 1) { setActive = true; setSel = 0; }
}

// ---------------------- примитивы рисования --------------------------
// Рисование обрезается вручную: на AVR координаты U8g2 беззнаковые 8-битные.
int16_t CLIP0 = 0;               // верхняя граница видимой области (для каруселей)

void gBox(int x, int y, int w, int h) {
  MEM_SAMPLE();
  int x0 = x < 0 ? 0 : x, x1 = x + w > 128 ? 128 : x + w;
  int y0 = y < CLIP0 ? CLIP0 : y, y1 = y + h > 64 ? 64 : y + h;
  if (x1 > x0 && y1 > y0) u8g2.drawBox(x0, y0, x1 - x0, y1 - y0);
}
void gHLine(int x, int y, int w) { gBox(x, y, w, 1); }
void gVLine(int x, int y, int h) { gBox(x, y, 1, h); }
void gFrame(int x, int y, int w, int h) {
  gHLine(x, y, w); gHLine(x, y + h - 1, w);
  gVLine(x, y, h); gVLine(x + w - 1, y, h);
}
void gStr(int x, int y, const char* s) {  MEM_SAMPLE();          // строка из ОЗУ
  if (x < 0 || y < 0 || y > 120) return;
  u8g2.drawUTF8(x, y, s);
}
void gStrP(int x, int y, const char* p) {        // строка из flash
  char b[28];
  strncpy_P(b, p, sizeof(b) - 1); b[sizeof(b) - 1] = 0;
  gStr(x, y, b);
}

void fmtHMS(char* b, uint32_t s) {
  uint32_t h = s / 3600, m = (s / 60) % 60, sc = s % 60;
  b[0] = '0' + (h / 10) % 10; b[1] = '0' + h % 10; b[2] = ':';
  b[3] = '0' + m / 10;        b[4] = '0' + m % 10; b[5] = ':';
  b[6] = '0' + sc / 10;       b[7] = '0' + sc % 10; b[8] = 0;
}

// одна строка битовой картинки шириной w бит, масштаб s
void drawBits(int x, int y, uint8_t bits, uint8_t w, int s) {
  uint8_t c = 0;
  while (c < w) {                                  // склеиваем подряд идущие пиксели
    if (bits & (1 << (w - 1 - c))) {
      uint8_t c2 = c;
      while (c2 + 1 < w && (bits & (1 << (w - 1 - (c2 + 1))))) c2++;
      gBox(x + c * s, y, (c2 - c + 1) * s, s);
      c = c2 + 1;
    } else c++;
  }
}
void drawGlyph(int x, int y, uint8_t g, int s) {
  for (uint8_t r = 0; r < 7; r++) drawBits(x, y + r * s, pgm_read_byte(&GL[g][r]), 5, s);
}
void drawDigit(int x, int y, uint8_t d, int s) {
  for (uint8_t r = 0; r < 6; r++) drawBits(x, y + r * s, pgm_read_byte(&DIG[d][r]), 5, s);
}

void drawFlame(int x, int y) {                    // 7x8 → 10x12 (масштаб 1.5)
  for (uint8_t j = 0; j < 8; j++) {
    uint8_t row = pgm_read_byte(&FLAME[j]);
    for (uint8_t i = 0; i < 7; i++)
      if (row & (1 << (6 - i))) {
        int x1 = i * 3 / 2, x2 = (i + 1) * 3 / 2;
        int y1 = j * 3 / 2, y2 = (j + 1) * 3 / 2;
        gBox(x + x1, y + y1, x2 > x1 ? x2 - x1 : 1, y2 > y1 ? y2 - y1 : 1);
      }
  }
}

// ЧЧ:ММ:СС цифрами высотой ровно 12 px, правый край x=116, строки 52..63
void drawCountdown(uint32_t s) {
  uint32_t h = s / 3600, m = (s / 60) % 60, sc = s % 60;
  uint8_t d[6] = {(uint8_t)((h / 10) % 10), (uint8_t)(h % 10),
                  (uint8_t)(m / 10), (uint8_t)(m % 10),
                  (uint8_t)(sc / 10), (uint8_t)(sc % 10)};
  int x = 116 - 84;                                // 6 цифр по 12 + 2 двоеточия по 6
  for (uint8_t i = 0; i < 6; i++) {
    drawDigit(x, 52, d[i], 2);
    x += 12;
    if (i == 1 || i == 3) {                        // двоеточие
      gBox(x + 2, 54, 2, 2);
      gBox(x + 2, 60, 2, 2);
      x += 6;
    }
  }
}

void drawCross(int x, int y, bool inv) {           // крестик 7x7
  u8g2.setDrawColor(inv ? 0 : 1);
  for (uint8_t k = 0; k < 7; k++) { gBox(x + k, y + k, 1, 1); gBox(x + 6 - k, y + k, 1, 1); }
  u8g2.setDrawColor(1);
}
void drawPlus(int cx, int cy, bool inv) {          // плюс 7x7
  u8g2.setDrawColor(inv ? 0 : 1);
  gBox(cx - 3, cy, 7, 1); gBox(cx, cy - 3, 1, 7);
  u8g2.setDrawColor(1);
}

// ------------------------- экранные элементы -------------------------
// Заголовок строго в жёлтой зоне (строки 0..15): шрифт 7x13, базовая линия 12.
void drawHeaderR(const char* title, uint8_t page) {        // заголовок из ОЗУ
  u8g2.setFont(F_TXT);
  gStr(0, 12, title);
  if (page > 0) {
    char b[4] = {(char)('1' + page), '/', '3', 0};
    gStr(127 - u8g2.getUTF8Width(b), 12, b);
  }
}
void drawHeaderP(const char* title, uint8_t page) {        // заголовок из flash
  char b[28];
  strncpy_P(b, title, sizeof(b) - 1); b[sizeof(b) - 1] = 0;
  drawHeaderR(b, page);
}
void drawDayHeader(const char* suffixP) {                  // «ПН ТАЙМЕРЫ»
  char b[28];
  strcpy_P(b, DSU[tSel]);
  strcat(b, " ");
  strcat_P(b, suffixP);
  drawHeaderR(b, desk);
}

// Пункт белой зоны высотой 12 px: i = 0..3
void listRow(uint8_t i, const char* labelP, const char* val, bool sel) {
  int top = YELLOW_H + ROW_H * i;
  u8g2.setFont(F_TXT);
  if (sel) { gBox(0, top, 126, ROW_H); u8g2.setDrawColor(0); }
  gStrP(1, top + 10, labelP);
  gStr(126 - u8g2.getUTF8Width(val), top + 10, val);
  u8g2.setDrawColor(1);
}

// карусель из дней недели; counts — показывать число таймеров
void drawCarousel(uint8_t v, int16_t offQ, bool counts) {
  CLIP0 = YELLOW_H;                                          // не заходим в жёлтую зону
  for (int8_t j = -2; j <= 2; j++) {
    int32_t uq = (int32_t)j * 256 + offQ;
    int32_t au = uq < 0 ? -uq : uq;
    int32_t sfq = 1152 - (au * 614) / 256;                  // 4.5 − 2.4·|u|
    if (sfq < 358) sfq = 358;                               // не меньше 1.4
    int s = (int)(sfq >> 8); if (s < 1) s = 1;
    uint8_t di = (v + j + 14) % 7;
    int xc = 64 + (int)((uq * 50) / 256);
    int w = 11 * s;
    int yt = 40 - (7 * s + 1) / 2;                          // центр белой зоны
    drawGlyph(xc - w / 2,         yt, pgm_read_byte(&DAYG[di][0]), s);
    drawGlyph(xc - w / 2 + 6 * s, yt, pgm_read_byte(&DAYG[di][1]), s);
    if (counts && s >= 2) drawDigit(xc + w / 2 + 2, yt + 7 * s - 3, cfg.cnt[di], 1);
  }
  gFrame(30, 20, 69, 41);
  CLIP0 = 0;
}

void drawEditor(const char* titleP) {
  drawHeaderP(titleP, desk);
  uint32_t v = edGet();
  uint8_t g[3] = {(uint8_t)(v / 3600), (uint8_t)((v / 60) % 60), (uint8_t)(v % 60)};
  u8g2.setFont(F_EDIT);
  int wd = u8g2.getStrWidth("00"), wc = u8g2.getStrWidth(":");
  int x0 = (128 - (3 * wd + 2 * wc)) / 2;
  bool hide = edBlink != 0 && ((int32_t)(edBlink - millis()) > 0) && ((millis() / 200) & 1);
  if (!hide) {
    for (uint8_t f = 0; f < 3; f++) {
      char b[3] = {(char)('0' + g[f] / 10), (char)('0' + g[f] % 10), 0};
      int x = x0 + f * (wd + wc);
      gStr(x, 47, b);
      if (f < 2) gStr(x + wd, 47, ":");
    }
  }
  gBox(x0 + edF * (wd + wc), 50, wd, 2);               // подчёркивание разряда
}

void drawDayList() {
  drawDayHeader(H_TIMERS);
  uint8_t c = cfg.cnt[tSel];
  int n = (int)dayRows(c) - dsTop; if (n > 4) n = 4;
  for (int k = 0; k <= n; k++) gHLine(0, YELLOW_H + ROW_H * k, 128);   // линии 1 px
  gVLine(0, YELLOW_H, ROW_H * n); gVLine(127, YELLOW_H, ROW_H * n);
  u8g2.setFont(F_TXT);
  for (int i = 0; i < n; i++) {
    int idx = dsTop + i, yy = YELLOW_H + ROW_H * i, yc = yy + 2;
    bool on = (idx == dsSel);
    if (idx == c) {                                              // строка «+»
      if (on) gBox(2, yc, 124, 9);
      drawPlus(64, yc + 4, on);
    } else {
      gVLine(102, yy, 13);
      bool tOn = on && !dsDel;
      if (tOn) gBox(2, yc, 99, 9);
      char b[9]; fmtHMS(b, cfg.t[tSel][idx].s);
      u8g2.setDrawColor(tOn ? 0 : 1);
      gStr(2, yy + 10, b);
      u8g2.setDrawColor(1);
      if (on && dsDel) gBox(104, yc, 22, 9);
      drawCross(112, yc + 1, on && dsDel);
    }
  }
}

void drawTimerMenu() {
  drawDayHeader(H_TIMER);
  char b[9];
  fmtHMS(b, cfg.t[tSel][tmIdx].s); listRow(0, L_START, b, tmSel == 0);
  fmtHMS(b, cfg.t[tSel][tmIdx].d); listRow(1, L_DUR,   b, tmSel == 1);
}

void drawTimerBar() {
  uint32_t now = millis();
  if (tActive) {
    int32_t rem = (int32_t)(tEnd - now);
    drawCountdown(rem <= 0 ? 0UL : (uint32_t)((rem + 999) / 1000));
    drawFlame(118, 52);
  } else if (tDoneUntil != 0 && (int32_t)(tDoneUntil - now) > 0 && ((now / 500) & 1)) {
    drawCountdown(0);
  }
}

// ------------------------------ столы --------------------------------
void drawScreen() {
  u8g2.setFontMode(1);
  u8g2.setDrawColor(1);
  CLIP0 = 0;
  if (desk == 0) {                                     // стол 1: часы 30 px (строки 19..48)
    uint32_t h = nowSec / 3600, m = (nowSec / 60) % 60, s = nowSec % 60;
    char a[3] = {(char)('0' + h / 10), (char)('0' + h % 10), 0};
    char b[3] = {(char)('0' + m / 10), (char)('0' + m % 10), 0};
    u8g2.setFont(F_CLOCK);
    int wd = u8g2.getStrWidth("00"), wc = u8g2.getStrWidth(":");
    int x0 = (128 - (2 * wd + wc)) / 2;
    gStr(x0, 49, a);
    if (!(s & 1)) gStr(x0 + wd, 49, ":");               // мигающее двоеточие
    gStr(x0 + wd + wc, 49, b);
    drawTimerBar();                                    // 12 px, строки 52..63
  } else if (desk == 1) {                               // стол 2: настройки
    if (setActive && setSub == 3) {
      drawHeaderP(H_DOW, 1);
      drawCarousel(dowNow(), carOffsetQ(dowAn), false);
    } else if (setActive && setSub != 0) {
      drawEditor(setSub == 1 ? H_TIME : H_TIMER);
    } else {
      drawHeaderP(H_SETTINGS, 1);
      char b[9];
      fmtHMS(b, nowSec);        listRow(0, L_TIME,  b, setActive && setSel == 0);
      fmtHMS(b, cfg.timerSec);  listRow(1, L_TIMER, b, setActive && setSel == 1);
      strcpy_P(b, DSL[dowNow()]);
      listRow(2, L_DOW, b, setActive && setSel == 2);
    }
  } else {                                              // стол 3: таймеры по дням
    if (tLevel == 0) {
      drawHeaderP(H_TIMERS, 2);
      drawCarousel(tSel > 6 ? dowNow() : tSel, carOffsetQ(tAn), true);
    } else if (tLevel == 1) drawDayList();
    else if (tLevel == 2) drawTimerMenu();
    else drawEditor(edKind == ED_START ? H_START : H_DUR);
  }
}

// ------------------------------ основной -----------------------------
void updateState() {
  uint32_t now = millis();
  if (dowAn.on && (now - dowAn.t0) >= 220) dowAn.on = false;
  if (tAn.on && (now - tAn.t0) >= 220) tAn.on = false;
  if (tActive && (int32_t)(tEnd - now) <= 0) { tActive = false; tDoneUntil = now + 5000UL; LOGMSG("TMR END"); }
  if (tSel > 6) tSel = dowNow();
}

// ------------------------ диагностика кнопок -------------------------
// Включается удержанием ⏱ при подаче питания. На экране 6 ячеек:
//   имя кнопки | уровень на выводе: 1 = разомкнута, 0 = замкнута
//   рамка = отпущена, залитая рамка = нажата (после фильтра)
//   «!» = кнопка залипла (замкнута дольше 20 с)
// Если кнопка нажата, а уровень остаётся 1 — обрыв/окисление контакта;
// если уровень 0 без нажатия — замыкание или наводка на линии.
void drawDiag() {
  u8g2.setFontMode(1);
  u8g2.setDrawColor(1);
  u8g2.setFont(F_TXT);
  gStrP(0, 12, H_DIAG);
  for (uint8_t i = 0; i < 6; i++) {
    int x = (i % 3) * 43, y = YELLOW_H + (i / 3) * 24;
    char b[5];
    strcpy_P(b, BN[i]);
    gFrame(x, y, 42, 24);
    gStr(x + 3, y + 11, b);
    char l[2] = {(char)(digitalRead(BTN_PIN[i]) == LOW ? '0' : '1'), 0};
    gStr(x + 33, y + 11, l);
    if (stuckMask & (1 << i)) gStr(x + 22, y + 11, "!");
    if (downMask & (1 << i)) gBox(x + 3, y + 14, 36, 7);
    else gFrame(x + 3, y + 14, 36, 7);
  }
}

void runDiag() {
  bool armed = false;
  uint32_t holdT = 0;
  for (;;) {
#if !BTN_ISR
    buttonsTickLoop();
#endif
    while (evH != evT) evH = (uint8_t)((evH + 1) & (EV_Q - 1));   // события в тесте не нужны
    if (!(downMask & (1 << 5))) { armed = true; holdT = 0; }       // ⏱ отпущена
    else if (armed) {
      if (!holdT) holdT = millis() | 1;
      else if ((millis() - holdT) > 2000UL) break;                 // удержание 2 с — выход
    }
    u8g2.firstPage();
    do {
      drawDiag();
#if !BTN_ISR
      buttonsTickLoop();
#endif
    } while (u8g2.nextPage());
  }
}

void showError(const char* msgP) {
  u8g2.firstPage();
  do { u8g2.setFont(F_TXT); u8g2.setFontMode(1); gStrP(4, 40, msgP); } while (u8g2.nextPage());
}

void setup() {
#if USE_LOG
  Serial.begin(LOG_BAUD);
  logBoot();
#endif
  for (uint8_t i = 0; i < 6; i++) pinMode(BTN_PIN[i], INPUT_PULLUP);
  pinMode(PIN_OUT, OUTPUT);
  outWrite(false);
  pinMode(PIN_RESET, INPUT);                           // в покое Hi-Z, не мешает линии RESET

  // Настроить тайм-аут до первого обращения к OLED по аппаратной I2C.
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000UL, true);
#endif
  LOGS("SETUP oled");
  u8g2.begin();                                        // сам запускает Wire (A4/A5)
  u8g2.setBusClock(400000);

#if BTN_ISR
  // Независимый аппаратный тик 1 мс для опроса кнопок.
  // Не изменяем Timer0, иначе ломаются millis()/delay()/PWM.
  TCCR2A = _BV(WGM21);                                // CTC
  TCCR2B = _BV(CS22);                                 // prescaler 64
  TCNT2  = 0;
  OCR2A  = 249;                                       // 1 ms при 16 MHz
  TIFR2  = _BV(OCF2A);                                // сбросить pending flag
  TIMSK2 = _BV(OCIE2A);                               // только Compare A
#endif

  if (digitalRead(PIN_TIMER) == LOW) { LOGS("SETUP key test"); runDiag(); }   // питание с зажатой ⏱ → тест кнопок

  LOGS("SETUP rtc");
  while (!swAck(RTC_ADDR)) { LOGS("SETUP RTC NOT FOUND"); showError(E_NORTC); delay(500); }
  if (dsLostPower()) dsAdjust(DateTime(F(__DATE__), F(__TIME__)));
  LOGS("SETUP cfg");
  loadCfg();
  readRtc();
  tSel = dowNow();
  LOGS("SETUP done");
}

void loop() {
#if !BTN_ISR
  buttonsTickLoop();
#endif
  LOG_LOOP();                                          // замер прохода, heartbeat
  uint32_t now = millis();
  if (now - lastRtc >= 100) {
    lastRtc = now;
    T_BEGIN(tr);
    readRtc();
    schedTick();
    T_MAX(mxRtc, tr);
  }
  i2cWatch();                                          // связь с RTC/OLED потеряна > 5 с → сброс через A2

  while (evH != evT) {                                 // события нажатий из очереди
    uint8_t i = evK[evH];
    evH = (uint8_t)((evH + 1) & (EV_Q - 1));
    LOG_KEY(i);
    onKey(BTN_KEY[i], 1);
    nextRep[i] = millis() + REPEAT_DELAY_MS;
  }
  for (uint8_t i = 0; i < 6; i++) {                    // автоповтор ▲/▼ в редакторах: шаг 10
    if ((downMask & (1 << i)) && editingKey(BTN_KEY[i]) && (int32_t)(millis() - nextRep[i]) >= 0) {
      onKey(BTN_KEY[i], 10);
      nextRep[i] = millis() + REPEAT_PERIOD_MS;
    }
  }

  updateState();
  outWrite(tActive);                                   // выход = индикатор событий
  saveStep();

  T_BEGIN(td);
  u8g2.firstPage();
  do {
    drawScreen();
#if !BTN_ISR
    buttonsTickLoop();
#endif
  } while (u8g2.nextPage());
  T_MAX(mxDraw, td);
}
