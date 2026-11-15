# Bill of materials

Prices checked **2026-11-14**, USD, before tax and shipping. Items sold in packs are charged at the share one robot uses
(the "per robot" column); the "unit" column is what you pay at checkout. Regenerate with `python docs/make_bom.py`.

| # | Part | Qty | Unit / pack price | Per robot | Role |
|---|---|---|---|---|---|
| 1 | [Feetech STS3215-C001 serial bus servo, 7.4 V, 19.5 kg-cm, magnetic encoder](https://www.alibaba.com/product-detail/Feetech-STS3215-19KG-7-4V-Dual_1601651378267.html) | 11 | $13.89 (2-499 pcs price) | **$152.79** | 10 leg joints + dome yaw |
| 2 | [Seeed XIAO ESP32S3 Sense (camera, mic, 8 MB PSRAM)](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html) | 1 | $19.99 | **$19.99** | brain, camera eye, ears, Wi-Fi |
| 3 | [Waveshare Bus Servo Adapter (A)](https://www.waveshare.com/product/bus-servo-adapter-a.htm) | 1 | $4.99 | **$4.99** | UART to half-duplex servo bus, power distribution |
| 4 | [1.28" round GC9A01 240x240 IPS LCD](https://www.amazon.com/Display-Module-Interface-240x240-Arduino/dp/B0H8P12XMC) | 1 | $6.24 | **$6.24** | the face / eye |
| 5 | [MPU-6050 GY-521 IMU](https://www.amazon.com/Kureelect-GY-521-Accelerometer-Compatible-Microcontroller/dp/B0H74JW12P) | 1 | $8.59 (3-pack) | **$2.86** | balance + fall detection |
| 6 | [Pololu D36V6F5 5 V 600 mA step-down regulator](https://www.pololu.com/product/3792) | 1 | $6.95 | **$6.95** | 5 V for XIAO + LCD (sits in the head) |
| 7 | [LiFePO4 IFR18650 3.2 V 1500 mAh cell](https://onlybatteries.com/3-2-volt-lifepo4-18650-battery-1500-mah/) | 2 | $10.69 | **$21.38** | 2S pack, 6.4 V nominal / 7.3 V full |
| 8 | [2S LiFePO4 protection board (BMS)](https://www.amazon.com/Midzoo-Protection-Charging-Recovery-LightPower/dp/B0982CLQCS) | 1 | $6.33 | **$6.33** | over-charge/discharge/short protection |
| 9 | [Passive piezo buzzer](https://www.amazon.com/MusRock-Frequency-Controllable-Electronic-Projects/dp/B0GXDX6ZY4) | 1 | $9.02 (10-pack) | **$0.90** | beeps and chirps |
| 10 | [5.5x2.1 mm panel DC jack](https://www.amazon.com/CESS-Socket-Threaded-Connector-Adapter/dp/B07FK6MMGH) | 1 | $6.79 (10-pack) | **$0.68** | charge port |
| 11 | [SS12D00 slide switch](https://www.amazon.com/dp/B07QC8HHCZ) | 1 | $10.29 (10-pack) | **$1.03** | power switch |
| 12 | [Copper foil tape (5 cm used)](https://www.amazon.com/MWRF-Source-Copper-Conductive-Adhesive/dp/B09FJ549C9) | 1 | $8.99 (10 m roll) | **$0.10** | capacitive touch pad inside the dome |
| 13 | [Silicone wire, 26 AWG (~2 m used)](https://www.amazon.com/BNTECHGO-16-30-Gauge-Silicone-colors/dp/B01M9800TG) | 1 | $12.48 (multi-colour kit) | **$1.00** | harness |
| 14 | [M2/M3 screws + M3 heat-set inserts (~100 pcs used)](https://www.amazon.com/Mardatt-Threaded-Stainless-Printing-Components/dp/B0DRVJVQSD) | 1 | $15.41 (kit, ~1/2 used) | **$7.71** | fasteners |
| 15 | [PLA filament (~360 g incl. 10% waste)](https://us.store.bambulab.com/products/pla-basic-filament) | 1 | $19.99 (1 kg refill) | **$7.20** | 14 printed parts |

**Total per robot: $240.15**  (servos are 64% of it)

## Notes

- **Servo price.** $13.89 is Feetech's own listing for 2-499 pieces. One-off retail for the same part is $21.99 (Waveshare ST3215). With retail servos the total is **$329.25**, which is over the $300 cap. The design price assumes buying servos direct from Feetech, which also allows buying in volume.
- **Charger, not in the total:** [Bioenno 7.3 V 2 A LiFePO4 charger, 5.5x2.1 DC plug](https://www.bioennopower.com/products/7-3v-2a-ac-to-dc-charger-dc-plug-for-6v-lifepo4-batteries-bpc-0602dc) $24.99. Any 7.3 V LiFePO4 charger with a 5.5x2.1 plug works. **Never use a Li-ion (8.4 V) charger** - it over-charges the cells and pushes the servos past their 7.4 V rating.
- **Why LiFePO4 2S:** 6.4 V nominal, 7.3 V full. That sits inside the STS3215 4-7.4 V range with no regulator in the servo path. 2S Li-ion would hit 8.4 V when full.
- **Run time (estimate, not measured):** about 9.6 Wh. Standing draws ~3-4 W and walking ~10-15 W, so expect roughly 40-60 min of mixed play.
- The servo horns, horn screws and 3-pin daisy-chain cables come in the STS3215 box. No separate cables are needed.
- Total is hardware only: no tax, shipping, or printer time.
