#!/usr/bin/env python3
"""Single source of truth for the BOM. Writes docs/BOM.md and site/assets/bom.json.
Prices checked 2026-11-14 (USD, before tax/shipping). Pack items are charged at the per-robot share."""
import json, os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
plastic_g = json.load(open(os.path.join(ROOT, 'cad', 'reports', 'summary.json')))['plastic_g']
fil_g = round(plastic_g * 1.10)  # +10% for brims, purge, failed first layers

# (item, qty, unit price, pack note, per-robot cost, link, role)
B = [
 ('Feetech STS3215-C001 serial bus servo, 7.4 V, 19.5 kg-cm, magnetic encoder', 11, 13.89, '2-499 pcs price', None,
  'https://www.alibaba.com/product-detail/Feetech-STS3215-19KG-7-4V-Dual_1601651378267.html', '10 leg joints + dome yaw'),
 ('Seeed XIAO ESP32S3 Sense (camera, mic, 8 MB PSRAM)', 1, 19.99, '', None,
  'https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html', 'brain, camera eye, ears, Wi-Fi'),
 ('Waveshare Bus Servo Adapter (A)', 1, 4.99, '', None,
  'https://www.waveshare.com/product/bus-servo-adapter-a.htm', 'UART to half-duplex servo bus, power distribution'),
 ('1.28" round GC9A01 240x240 IPS LCD', 1, 6.24, '', None,
  'https://www.amazon.com/Display-Module-Interface-240x240-Arduino/dp/B0H8P12XMC', 'the face / eye'),
 ('MPU-6050 GY-521 IMU', 1, 8.59, '3-pack', 8.59/3,
  'https://www.amazon.com/Kureelect-GY-521-Accelerometer-Compatible-Microcontroller/dp/B0H74JW12P', 'balance + fall detection'),
 ('Pololu D36V6F5 5 V 600 mA step-down regulator', 1, 6.95, '', None,
  'https://www.pololu.com/product/3792', '5 V for XIAO + LCD (sits in the head)'),
 ('LiFePO4 IFR18650 3.2 V 1500 mAh cell', 2, 10.69, '', None,
  'https://onlybatteries.com/3-2-volt-lifepo4-18650-battery-1500-mah/', '2S pack, 6.4 V nominal / 7.3 V full'),
 ('2S LiFePO4 protection board (BMS)', 1, 6.33, '', None,
  'https://www.amazon.com/Midzoo-Protection-Charging-Recovery-LightPower/dp/B0982CLQCS', 'over-charge/discharge/short protection'),
 ('Passive piezo buzzer', 1, 9.02, '10-pack', 0.90,
  'https://www.amazon.com/MusRock-Frequency-Controllable-Electronic-Projects/dp/B0GXDX6ZY4', 'beeps and chirps'),
 ('5.5x2.1 mm panel DC jack', 1, 6.79, '10-pack', 0.68,
  'https://www.amazon.com/CESS-Socket-Threaded-Connector-Adapter/dp/B07FK6MMGH', 'charge port'),
 ('SS12D00 slide switch', 1, 10.29, '10-pack', 1.03,
  'https://www.amazon.com/dp/B07QC8HHCZ', 'power switch'),
 ('Copper foil tape (5 cm used)', 1, 8.99, '10 m roll', 0.10,
  'https://www.amazon.com/MWRF-Source-Copper-Conductive-Adhesive/dp/B09FJ549C9', 'capacitive touch pad inside the dome'),
 ('Silicone wire, 26 AWG (~2 m used)', 1, 12.48, 'multi-colour kit', 1.00,
  'https://www.amazon.com/BNTECHGO-16-30-Gauge-Silicone-colors/dp/B01M9800TG', 'harness'),
 ('M2/M3 screws + M3 heat-set inserts (~100 pcs used)', 1, 15.41, 'kit, ~1/2 used', 7.71,
  'https://www.amazon.com/Mardatt-Threaded-Stainless-Printing-Components/dp/B0DRVJVQSD', 'fasteners'),
 (f'PLA filament (~{fil_g} g incl. 10% waste)', 1, 19.99, '1 kg refill', round(19.99 * fil_g / 1000, 2),
  'https://us.store.bambulab.com/products/pla-basic-filament', '14 printed parts'),
]
rows = []
for item, q, unit, note, per, link, role in B:
    cost = round(per if per is not None else q * unit, 2)
    rows.append(dict(item=item, qty=q, unit_usd=unit, pack=note, robot_usd=cost, link=link, role=role))
total = round(sum(r['robot_usd'] for r in rows), 2)
servo = next(r for r in rows if 'STS3215' in r['item'])
charger = dict(item='Bioenno 7.3 V 2 A LiFePO4 charger, 5.5x2.1 DC plug', usd=24.99,
               link='https://www.bioennopower.com/products/7-3v-2a-ac-to-dc-charger-dc-plug-for-6v-lifepo4-batteries-bpc-0602dc')
retail_servo = dict(note='Same servo at retail, 1 pc: Waveshare ST3215 $21.99', link='https://www.waveshare.com/st3215-servo.htm?sku=22414', usd=21.99)
total_retail_servo = round(total - servo['robot_usd'] + 11 * 21.99, 2)
out = dict(checked='2026-11-14', currency='USD', rows=rows, total_usd=total, servo_share=round(servo['robot_usd'] / total, 3),
           charger=charger, retail_servo=retail_servo, total_with_retail_servos_usd=total_retail_servo)
os.makedirs(os.path.join(ROOT, 'site', 'assets'), exist_ok=True)
json.dump(out, open(os.path.join(ROOT, 'site', 'assets', 'bom.json'), 'w'), indent=1)

md = ['# Bill of materials', '',
      'Prices checked **2026-11-14**, USD, before tax and shipping. Items sold in packs are charged at the share one robot uses',
      '(the "per robot" column); the "unit" column is what you pay at checkout. Regenerate with `python docs/make_bom.py`.', '',
      '| # | Part | Qty | Unit / pack price | Per robot | Role |', '|---|---|---|---|---|---|']
for i, r in enumerate(rows, 1):
    pk = f" ({r['pack']})" if r['pack'] else ''
    md.append(f"| {i} | [{r['item']}]({r['link']}) | {r['qty']} | ${r['unit_usd']:.2f}{pk} | **${r['robot_usd']:.2f}** | {r['role']} |")
md += ['', f"**Total per robot: ${total:.2f}**  (servos are {100*out['servo_share']:.0f}% of it)", '',
       '## Notes', '',
       f"- **Servo price.** $13.89 is Feetech's own listing for 2-499 pieces. One-off retail for the same part is "
       f"$21.99 (Waveshare ST3215). With retail servos the total is **${total_retail_servo:.2f}**, which is over the $300 cap. "
       "The design price assumes buying servos direct from Feetech, which also allows buying in volume.",
       f"- **Charger, not in the total:** [{charger['item']}]({charger['link']}) ${charger['usd']:.2f}. Any 7.3 V LiFePO4 charger with a 5.5x2.1 plug works. "
       "**Never use a Li-ion (8.4 V) charger** - it over-charges the cells and pushes the servos past their 7.4 V rating.",
       '- **Why LiFePO4 2S:** 6.4 V nominal, 7.3 V full. That sits inside the STS3215 4-7.4 V range with no regulator in the servo path. '
       '2S Li-ion would hit 8.4 V when full.',
       '- **Run time (estimate, not measured):** about 9.6 Wh. Standing draws ~3-4 W and walking ~10-15 W, so expect roughly 40-60 min of mixed play.',
       '- The servo horns, horn screws and 3-pin daisy-chain cables come in the STS3215 box. No separate cables are needed.',
       '- Total is hardware only: no tax, shipping, or printer time.']
open(os.path.join(ROOT, 'docs', 'BOM.md'), 'w').write('\n'.join(md) + '\n')
print('total', total, 'retail-servo total', total_retail_servo, 'filament g', fil_g)
