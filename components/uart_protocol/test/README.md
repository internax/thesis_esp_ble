# uart_protocol — testy

## Struktura

```
test/
├── unit/           Unit testy (calc_crc8, encode, decode) — běží na ESP32, nevyžadují HW zapojení
└── integration/    Integrační test UART komunikace — vyžadují dvě ESP32-S3 desky
```

---

## Unit testy

Testují čistou logiku bez závislosti na hardwaru.

| Soubor | Co testuje |
|---|---|
| `unit/main/test_crc8.cpp` | `calc_crc8` — správnost CRC-8/SMBUS výpočtu |
| `unit/main/test_encode_decode.cpp` | `encode` / `decode` — sestavení a parsování framu |

### Spuštění

```bash
cd components/uart_protocol/test/unit
idf.py set-target esp32s3
idf.py build flash monitor
```

Výstup při úspěchu:
```
16/16 Tests OK
```

### Přidání nového testovacího souboru

1. Vytvoř `unit/main/test_neco.cpp` s `TEST_CASE` makry
2. Přidej ho do `unit/main/CMakeLists.txt` do `SRCS`
3. `WHOLE_ARCHIVE` v CMakeLists.txt musí zůstat — bez něj linker zahodí object soubory
   na které nic explicitně neodkazuje, a jejich testy se nespustí

---

## Integrační test

Ověřuje reálnou UART komunikaci mezi dvěma deskami.

### Zapojení

```
Board A GPIO16 (TX) ──► Board B GPIO17 (RX)
Board A GPIO17 (RX) ◄── Board B GPIO16 (TX)
GND ◄──────────────────────────────────► GND
```

### Flash

V `integration/main/main.cpp` nastav roli pomocí `#define`:

| `#define` | Role |
|---|---|
| `UART_ROLE_SENDER` | Posílá frame každé 2s, čeká na ACK |
| `UART_ROLE_RECEIVER` | Přijímá frame, odesílá ACK |

```bash
cd components/uart_protocol/test/integration
idf.py set-target esp32s3
idf.py build flash monitor
```

Flashni každou desku zvlášť se správným `#define`.

### Očekávaný výstup

**Sender:**
```
Sending frame...
ACK OK
```

**Receiver:**
```
Received cmd=0x10 data=0x01  MAC=FF:EE:DD:CC:BB:AA
ACK sent
```
