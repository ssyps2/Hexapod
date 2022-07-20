import test_kine, time

hexapod = test_kine.hex_kine(ctrl_freq = 30)

hexapod.initHexapod()
print('init...')

while True:
    hexapod.cmdHexapodMove(0.1,0,0,test_kine.hex_mode_e.TRIPOD)
    time.sleep(1/30)
    