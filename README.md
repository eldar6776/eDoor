# eDoor

Sistem za kontrolu vrata baziran na **125 kHz RFID čitaču** i **RS485 komunikacionoj sabirnici**. Repo sadrži firmware za mikrokontroler, hardverski dizajn ploče, desktop alat za konfiguraciju i prateću dokumentaciju za razvoj RFID/door access uređaja.

## Šta je ovo

`eDoor` je embedded/hardware projekat za kontrolu pristupa. Uređaj očitava 125 kHz RFID kartice, provjerava ih prema listi sačuvanoj u EEPROM memoriji, evidentira događaje i upravlja bravom preko lokalne logike uređaja. Komunikacija i nadzor se rade preko **RS485** magistrale, a u repozitoriju postoji i Windows aplikacija za slanje komandi kontroleru.

Na osnovu sadržaja repozitorija, projekat obuhvata:

- **firmware** za STM32 mikrokontroler
- **RFID logiku** baziranu na HTRC110 / EM4102 125 kHz pristupu
- **upravljanje memorijom** kartica i logova u EEPROM-u
- **RS485 komandni protokol** za konfiguraciju i nadzor
- **desktop konfigurator** za Windows
- **PCB/schematic** fajlove i fabrication output pakete
- **datasheetove i referentnu dokumentaciju** za razvoj hardvera

## Glavne funkcionalnosti

- očitavanje 125 kHz RFID kartica
- validacija kartica iz memorije uređaja
- vođenje evidencije događaja
- upravljanje bravom vrata
- uključivanje/isključivanje buzzera
- podešavanje vremena uređaja
- dodavanje i brisanje kartica
- brisanje logova događaja
- restart pojedinačnog uređaja ili više uređaja preko RS485 mreže
- PC alat za testiranje i administraciju komandi

## Struktura repozitorija

```text
.
├── doc/   # datasheetovi, reference design dokumenti, slike, RS485 protokol
├── fw/    # firmware projekti i build artefakti
├── hw/    # Altium hardware projekti, PCB, šeme i fabrication output
├── sw/    # desktop software za komunikaciju sa uređajem
└── README.md
```

### `fw/`
Firmware direktorij sadrži više verzija projekta (`DE-071124`, `DE-100924`, `DE-290624`) i arhivu izvornog koda.

Najkompletnija novija struktura vidi se u `fw/DE-100924/`:

- `Core/` – glavna aplikaciona logika
- `Drivers/` – STM32 HAL i CMSIS
- `Common/` – zajednički pomoćni kod
- `MDK-ARM/` – Keil/MDK-ARM projekat i build fajlovi
- `DE-100924.ioc` – STM32CubeMX konfiguracija

Iz sadržaja fajlova može se zaključiti da firmware koristi:

- **STM32F0** platformu
- **Keil MDK-ARM** toolchain
- **STM32CubeMX** za generisanje perifernih postavki
- **I2C EEPROM** za pohranu kartica i logova
- posebne module poput:
  - `rfid.c/.h`
  - `eeprom.c/.h`
  - `logger.*`
  - `main.*`

### `hw/`
Hardverski dio sadrži više revizija PCB projekta, uključujući fajlove kao što su:

- `.SchDoc` – šeme
- `.PcbDoc` – PCB layout
- `.PrjPcb` – Altium projekat
- `.BomDoc` – bill of materials dokumenti
- `.pdf` i `3D.pdf` – izvezeni pregledi dizajna
- fabrication zip pakete za proizvodnju

To ukazuje da je projekat razvijan u **Altium Designer** okruženju.

### `sw/`
Softverski alat u `sw/rfid/` je **Windows Forms** aplikacija u C#.

Karakteristike:

- Visual Studio solution: `sw/rfid/rfid.sln`
- glavna aplikacija: `sw/rfid/RubiconCtrlConf/`
- target framework: **.NET Framework 4.8**
- serijska komunikacija preko COM porta
- izbor komandi preko GUI interfejsa
- prikaz odgovora kontrolera i real-time događaja

Aplikacija omogućava slanje komandi kao što su:

- `GET_SYS_FLAG`
- `GET_CARD_CNT`
- `GET_CARD_PRESENT`
- `GET_EVENT_CNT`
- `GET_EVENT_LAST`
- `SET_SYS_TIME`
- `SET_SYS_RESTART`
- `SET_CARD_ONE`
- `SET_DOOR_OPEN`
- `SET_DOOR_TIME`
- `SET_DOOR_ENABLE`
- `SET_DOOR_DISABLE`
- `SET_BUZZER_ENABLE`
- `SET_BUZZER_DISABLE`
- `DELETE_CARD_ONE`
- `DELETE_CARD_ALL`
- `DELETE_EVENT_LAST`
- `DELETE_EVENT_ALL`
- `RESTART_ONE`
- `RESTART_ALL`

### `doc/`
Dokumentacija uključuje datasheetove i referentne materijale vezane za:

- **HTRC110**
- 125 kHz RFID antene i analogni front-end
- RDM6300 reference
- MOSFET/zaštitne komponente
- napajanje i pomoćne sklopove
- slike prototipa i razvojne materijale
- tekstualni opis **RS485 protokola**

## Arhitektura sistema

Na visokom nivou projekat izgleda ovako:

1. RFID front-end očitava 125 kHz karticu.
2. Firmware dekodira i obrađuje ID kartice.
3. Kartica se poredi sa zapisima u EEPROM memoriji.
4. Ako je validna, aktivira se izlaz za bravu.
5. Događaj se upisuje u memoriju logova.
6. Sistem može biti nadziran i konfigurisan preko RS485 magistrale.
7. PC aplikacija šalje komande i čita odgovore uređaja.

## RS485 protokol

Repo sadrži opis protokola u fajlu `doc/rs485_protokol.txt`.

Osnovni format paketa je:

- **BAJT 0** = `SOH` (`0x01`)
- **BAJT 1** = adresa čitača
- **BAJT 2** = dužina paketa
- **BAJT 3** = komanda
- **BAJT 4...** = parametri (npr. broj kartice ili vrijeme)
- **zadnja 2 bajta prije kraja** = checksum
- **zadnji bajt** = `EOT` (`0x04`)

Komande definisane u dokumentaciji uključuju npr.:

- `GET_SYS_FLAG`
- `GET_CARD_CNT`
- `GET_CARD_ALL`
- `GET_EVENT_CNT`
- `GET_EVENT_LAST`
- `SET_SYS_FLAG`
- `SET_SYS_TIME`
- `SET_SYS_RESTART`
- `SET_CARD_ONE`
- `DELETE_CARD_ONE`
- `DELETE_CARD_ALL`
- `DELETE_EVENT_LAST`
- `DELETE_EVENT_ALL`

Napomena: desktop aplikacija i tekstualna dokumentacija pokazuju da je protokol vjerovatno evoluirao kroz više verzija, pa prije upotrebe treba uskladiti konkretan firmware build i PC alat.

## Firmware detalji

Iz dostupnog koda se vidi nekoliko važnih podsistema:

### RFID modul
Firmware sadrži `rfid.c` i `rfid.h`, gdje je implementirana state machine logika za inicijalizaciju i očitavanje RFID taga. Kod referencira **HTRC110** način rada i obradu 125 kHz RFID ulaza.

### EEPROM modul
`eeprom.c/.h` implementira čuvanje podataka u eksternom EEPROM-u preko I2C-a. Definisani su posebni memorijski opsezi za:

- sistemsku konfiguraciju
- vrijeme brave
- listu RFID kartica
- listu logovanih događaja

### Logovanje i upravljanje događajima
Na osnovu imenovanja modula i komandnog interfejsa, firmware podržava brojanje, čitanje i brisanje događaja, što je tipično za access-control uređaje.

## Hardverski detalji

Po strukturi repozitorija i priloženoj dokumentaciji, hardver vjerovatno uključuje:

- STM32 mikrokontroler
- 125 kHz RFID analogni front-end / reader stage
- RS485 komunikacioni interfejs
- izlaz za upravljanje bravom
- buzzer i signalizaciju
- eksterni EEPROM
- pomoćne zaštitne i napojne komponente

Tačan spisak komponenti i konekcija treba provjeriti u `hw/DE-150824/` i PDF izvozu šema/PCB-a.

## Kako otvoriti projekat

### Firmware
Za firmware će ti najvjerovatnije trebati:

- **Keil MDK-ARM**
- **STM32CubeMX**
- po potrebi **STM32CubeProgrammer** za programiranje MCU-a

Koraci:

1. Otvori `fw/DE-100924/DE-100924.ioc` u STM32CubeMX ako želiš pregled konfiguracije periferija.
2. Otvori Keil projekat unutar `fw/DE-100924/MDK-ARM/` za build i debug.
3. Pregledaj `Core/` i `Common/` za aplikacionu logiku.

### Hardware
Za hardverski dio će ti trebati:

- **Altium Designer**

Otvori odgovarajući projekat unutar npr. `hw/DE-150824/`.

### Desktop aplikacija
Za PC alat će ti trebati:

- **Visual Studio**
- **.NET Framework 4.8 Developer Pack**

Koraci:

1. Otvori `sw/rfid/rfid.sln`
2. Buildaj projekat `RubiconCtrlConf`
3. Poveži se na odgovarajući COM port
4. Podesi baudrate i RS485 adrese
5. Pošalji željenu komandu uređaju

## Status repozitorija

Repo izgleda kao **realan razvojni repozitorij proizvoda/prototipa**, a ne kao ispoliran open-source paket. To znači da sadrži i:

- više hardverskih i firmware revizija
- build artefakte
- zip arhive
- vendor drivere
- bin/obj/.vs foldere
- internu projektnu dokumentaciju

Zbog toga je dobar za razvoj i arhiviranje projekta, ali bi za širu open-source upotrebu vjerovatno koristilo dodatno čišćenje strukture i dopuna dokumentacije.

## Preporuke za dalje

Ako želiš da repo bude lakši za korištenje drugima, korisno bi bilo dodati još:

- blok dijagram sistema
- fotografije gotove ploče i konektora
- listu podržanih RFID kartica/tagova
- mapu pinova i konektora
- opis napajanja i izlaza brave
- jasnu proceduru za flash firmware-a
- primjer RS485 paketa zahtjev/odgovor
- changelog po hardverskim/firmware revizijama

## Napomena

README je sastavljen analizom dostupne strukture repozitorija, konfiguracionih fajlova i dijela izvornog koda. Neki detalji hardvera i tačan deployment postupak mogu se razlikovati po reviziji projekta, pa ih treba potvrditi direktno iz šema, PCB fajlova i firmware konfiguracije.
