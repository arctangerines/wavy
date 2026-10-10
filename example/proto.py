import numpy as np
import ctypes
import matplotlib.pyplot as plt

fname = ["july_3_2026_4pm.wavy",
"july_3_2026_39pm _30min.wavy",
"july_6_1315.wavy",
"july_6_1430.wavy",
"july_8_1412.wavy",
"july_9_12_utc.wavy",]
fname = ["samples/120304_130726_1"]

for file in fname :
    wavyf = open(file, 'rb')

    jdx = 0
    comps = [[], [], []]
    w = 0

    wavyd = wavyf.read(32)
    wflag = 0
    idx = 0
    while 1:
        wavyd = wavyf.read(3)
        if not wavyd:
            break

        w = int.from_bytes(wavyd)
        w = w >> 4
        if w & 0x80000:
            w = w | (0x0fff << 20)
        else:
            w + 1

        w = w & 0xffffffff
        w = ctypes.c_int(w).value
        comps[jdx].append(int(w))

        idx +=3
        if idx == 2250:
            wavyf.read(22)
            idx = 0
        jdx += 1
        if jdx == 3:
            jdx = 0

    sens = 3.9e-6
    gconv = 9.80665
    x = np.array(comps[0])
    x = x * sens
    # x = x * gconv


    y = np.array(comps[1])
    y = y * sens
    # y = y * gconv

    z = np.array(comps[2])
    z = z * sens
    # z = z * gconv

    t = np.zeros(len(x))
    print(len(x))
    print(len(y))
    print(len(z))

    with open(f"{file}_x_.txt", "w") as f:
        for i, v in enumerate(x):
            f.write(f"{v}\n")
    with open(f"{file}_y_.txt", "w") as f:
        for i, v in enumerate(y):
            f.write(f"{v}\n")
    with open(f"{file}_z_.txt", "w") as f:
        for i, v in enumerate(z):
            f.write(f"{v}\n")

    for i, v in enumerate(t):
        if i == 0:
            continue
        t[i] = t[i - 1] + 0.004

    fig, ax = plt.subplots()
    ax.plot(t, x)
    ax.grid()
    plt.show()

    fig, ax = plt.subplots()
    ax.plot(t, y)
    ax.grid()
    plt.show()

    fig, ax = plt.subplots()
    ax.plot(t, z)
    ax.grid()
    plt.show()
    print(comps)
