# ESP_32_receiver
 ESP-WROOM-32 module receiver for morse code

#Tikslas
Sukurti dvi tarpusavyje bendraujančias sistemas – siųstuvą ir imtuvą, galinčias perduoti trumpus
tekstinius pranešimus Morzės kodu, panaudoti ESP-WROOM-32 belaidį ryšį (Wi-Fi arba BLE)

#2 grupė – Imtuvas:
1. Priima signalą, dekoduoja jį į Morzės kodą, o vėliau – į tekstą.
2. Atlieka klaidų aptikimą / taisymą taikant Hamingo metodą.
3. Išmatuoja signalo stiprį (RSSI) ir pateikia jį dBm formatu.
4. Apskaičiuoja kanalo efektyvumas, klaidų tankį (BER).
5. Atvaizduoja rezultatus ekranėlyje
