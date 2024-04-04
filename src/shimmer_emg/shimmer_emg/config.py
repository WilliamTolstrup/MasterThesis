import serial
import struct

def wait_for_ack():
    ddata = b""
    ack = struct.pack('B', 0xff)
    while ddata != ack:
        ddata = serialObj.read(1)
    print("")
    return

def setSamplingRateHz(rate=512):
    # send the set sampling rate command
    print("Sending sampling rate command...\n")
    sampling_freq = rate #Hz
    clock_wait = (2 << 14) // sampling_freq # Double // for integer division (python3)
    serialObj.write(struct.pack('<BH', 0x05, clock_wait))
    print("Sampling rate: ")
    print(sampling_freq)
    wait_for_ack()

serialObj = serial.Serial()
serialObj.baudrate = 115200
serialObj.timeout = 1

port = str('/dev/rfcomm0')
print(port)
serialObj.port = port
serialObj.open()

samplingFrequency = 1000 # frequency in Hz
exgSamplingRate = 0x03


# Chip 1 configuration
exgGainValueCh1 = 0x69
exgGainValueCh2 = 0x60
chip1Config = [exgSamplingRate, 0xA0, 0x10, exgGainValueCh1, exgGainValueCh2, 0x00, 0x00, 0x00, 0x02, 0x03]

serialObj.flushInput()

# Get the daughter card ID byte (SR number)
print("Requesting Daughter Card ID and Revision number...")
serialObj.write(struct.pack('BBB', 0x66, 0x02,0x00))

wait_for_ack()

ddata = list(struct.unpack(4*'B', serialObj.read(4)))
srNumber = ddata[2]
srRev = ddata[3]

print("Device: SR%d-%d" % (srNumber, srRev))

print("Sending sensor commands for IMU and EMG communication...")
serialObj.write(struct.pack('BBBB', 0x08, 0xF0, 0x00, 0x00)) # 0x08 is command byte, 0xE0 enables IMU, 0x00 is trailing.
#serialObj.write(struct.pack('BBBB', 0x08, 0x10, 0x00, 0x00)) # 0x08 is command byte, 0x10 enables EMG, 0x00 is trailing.


wait_for_ack()

serialObj.write(struct.pack('BBB', 0x05, 0x80, 0x02)) #51.2Hz (32768/640=51.2Hz: 640 -> 0x0280; has to be done like this for alignment reasons.)
wait_for_ack()

if(srNumber == 47 and srRev >= 4):
    chip1Config[1] |= 8 # Config byte for CHIP1 in SR47-4

# Send the set sampling rate command
setSamplingRateHz(samplingFrequency)

# Configure Chip 1
print("Sending chip configuration...")
chip1Config = [0x61, 0x00, 0x00, 0x0A] + chip1Config
serialObj.write(chip1Config)
print("Chip configuration: ")
print(chip1Config)
print("")
wait_for_ack()

# send start streaming command
print("Start streaming command sent...\n")
print("")
serialObj.write(struct.pack('B', 0x07))
wait_for_ack()
print("Now streaming data.")

ddata = b""  # Use bytes literal for Python 3
numbytes = 0
framesize = 22 # 33

exgCalFactor = (((2.42 * 1000) / 12) / (pow(2, 23) - 1))

while True:
    #while numbytes < framesize:
    #     ddata += serialObj.read(framesize)
    #     numbytes = len(ddata)

     #data = ddata[0:framesize]
    # ddata = ddata[framesize:]
    # numbytes = len(ddata)

    # Read one byte at a time
    data = serialObj.read(size=1)

    # Check if the byte is not empty
    if data:
    # Convert the byte to its integer representation for easier viewing
        byte = int.from_bytes(data, 'big')
        print(byte, end=' ', flush=True)
    #print(data)
    # packettype = struct.unpack('B', data[0:1])
    # (ts0, ts1) = struct.unpack('BB', data[1:3])
    # (accx, accy, accz) = struct.unpack('HHH', data[3:9])
    # (gyrox, gyroy, gyroz) = struct.unpack('HHH', data[9:15])
    # (magx, magy, magz) = struct.unpack('>hhh', data[15:21])
    # #ch1 = struct.unpack('>i', data[21:24] + b'\0')[0] >> 8
    # #ch2 = struct.unpack('>i', data[24:27] + b'\0')[0] >> 8

    # #ch1 *= exgCalFactor
    # #ch2 *= exgCalFactor

    # print("")
    # print(packettype, ts0, ts1)
    # print(accx, accy, accz)
    # print(gyrox, gyroy, gyroz)
    # print(magx, magy, magz)
    #print(ch1, ch2)


    # (packettype,) = struct.unpack('B', data[0:1])
    # (timestamp,) = struct.unpack('H', data[1:3])
    # (accx, accy, accz) = struct.unpack('HHH', data[3:9])
    # (gyrox, gyroy, gyroz) = struct.unpack('HHH', data[9:15])
    # (magx, magy, magz) = struct.unpack('>hhh', data[15:21])
    # ch1 = struct.unpack('>i', data[21:24] + b'\0')[0] >> 8
    # ch2 = struct.unpack('>i', data[24:27] + b'\0')[0] >> 8

"""

EMG, IMU
0 197 136 4   13  7   11  7   36  7  
0 229 136 4   225 6   151 7   76  7
0 5   137 4   76  7   201 7   223 7
0 37  137 4   200 7   8   8   133 7
0 69  137 4   252 6   44  8   204 7
0 101 137 4   123 7   116 8   150 8
0 133 137 4   126 7   140 8   252 7
0 165 137 4   39  8   44  8   244 7
0 197 137 4   65  8   70  8   59  8
0 229 137 4   194 7   30  8   230 7
0 5   138 4   8   8   161 7   164 7
0 37  138 4   64  7   180 7   191 7
0 69  138 4   125 7   66  8   33  8
0 101 138 4   88  7   42  8   254 7
0 133 138 4   36  8   240 7   112 8
0 165 138 4   101 8   125 8   234 8
0 197 138 4   150 8   24  8   13  8
0 229 138 4   97  7   44  8   251 7

IMU
packet ts0, ts1,6 bytes accel, 6 bytes gyro, 6 bytes mag, ?, extra

0  1   2  3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20 21 (22 extra)  MATCHES OG IMU SCRIPT

0 163 33  2   23  8   16  8   204 4   0   53  0   54  255 251 250 0   252 255 71 1
0 227 33  2   23  8   17  8   206 4   0   36  0   50  255 230 245 0   252 255 66 1
0 67  34  2   23  8   15  8   207 4   0   41  0   29  255 240 245 0   252 255 66 1
0 131 34  2   23  8   15  8   206 4   0   55  0   41  255 248 245 0   252 255 66 1
0 195 34  2   24  8   16  8   206 4   0   40  0   52  255 241 245 0   252 255 66 1
0 3   35  2   22  8   16  8   206 4   0   20  0   43  255 234 245 0   252 255 66 1
0 67  35  2   22  8   16  8   205 4   0   44  0   33  255 252 251 0   248 255 66 1
0 131 35  2   22  8   16  8   205 4   0   51  0   33  255 242 251 0   248 255 66 1
0 195 35  2   23  8   16  8   205 4   0   33  0   42  255 249 251 0   248 255 66 1
0 3   36  2   23  8   15  8   205 4   0   49  0   55  255 246 251 0   248 255 66 1
0 67  36  2   22  8   14  8   205 4   0   31  0   39  255 242 251 0   248 255 66 1
0 131 36  2   22  8   14  8   207 4   0   36  0   46  255 241 253 0   254 255 62 1
0 195 36  2   21  8   15  8   208 4   0   46  0   43  255 249 253 0   254 255 62 1
0 3   37  2   23  8   16  8   206 4   0   36  0   51  255 241 253 0   254 255 62 1
0 67  37  2   23  8   15  8   208 4   0   40  0   26  0   1   253 0   254 255 62 1
0 131 37  2   23  8   15  8   207 4   0   40  0   34  255 248 253 0   254 255 62 1
0 195 37  2   23  8   16  8   206 4   0   50  0   47  255 235 252 0   252 255 71 1
0 3   38  2   23  8   17  8   206 4   0   48  0   42  0   6   252 0   252 255 71 1
0 67  38  2   24  8   16  8   206 4   0   44  0   39  0   0   252 0   252 255 71 1

EMG
packet, ts0, ts1, ts2, c1status, 3 byte ch1, 3 byte ch2, + extra

0   1  2   3   4   5   6   7  8    9   10 (11 extra) (matches OG emg script)

0  214 40  2  128 251 227 84  6   211 127
0  246 40  2  128 251 191 228 6   178 74
0  22  41  2  128 254 187 84  6   98  210
0  54  41  2  128 5   127 218 4   127 90
0  86  41  2  128 13  59  66  255 211 23
0  118 41  2  128 16  242 13  251 127 170
0  150 41  2  128 16  29  241 250 29  216
0  182 41  2  128 17  234 89  249 238 50
0  214 41  2  128 19  3   187 250 48  87
0  246 41  2  128 18  75  77  250 162 148
0  22  42  2  128 17  0   228 250 250 47
p  ttt tt  t  ttt ch1 ch1 ch2 ch2 ch2 ch2 

EMG, IMU
(Looks like just EMG?)
0 212 243 1 25  8 145 7 165 7
0 244 243 1 96  7 239 7 90  8
0 20  244 1 250 7 136 8 220 7
0 52  244 1 232 7 134 8 175 7
0 84  244 1 110 7 119 8 154 8
0 116 244 1 124 7 133 8 238 7
0 148 244 1 11  8 220 7 59  8
0 180 244 1 185 7 229 7 113 8
0 212 244 1 79  8 80  8 124 8
0 244 244 1 136 8 228 7 125 8
0 20  245 1 163 8 2   8 175 7


IMU, EMG
(Looks like just EMG?)
0 173 155 1 249 6 54  8 238 7
0 205 155 1 238 7 140 8 195 7
0 237 155 1 166 7 238 7 89  8
0 13  156 1 5   8 147 7 167 7
0 45  156 1 85  7 222 7 45  8
0 77  156 1 118 7 67  8 77  8
0 109 156 1 206 7 61  8 67  8
0 141 156 1 221 7 97  8 152 8
0 173 156 1 138 7 154 7 203 7
0 205 156 1 155 7 124 7 75  8
0 237 156 1 219 7 20  8 206 7

EMG + IMU (in same .write)

0 103 161 7 7   10  128 8 0   0  128 9   216 164 7   153 133
0 167 161 7 248 9   214 9 0   0  128 5   210 197 10  120 244
0 231 161 7 4   10  175 8 0   0  128 2   166 117 10  85  147
0 39  162 7 5   10  105 8 0   0  128 1   128 128 9   154 78
0 103 162 7 3   10  175 8 0   0  128 1   228 0   8   164 11
0 167 162 7 249 9   146 9 0   0  128 7   167 116 6   152 121
0 231 162 7 4   10  178 8 0   0  128 251 74  47  2   143 31
0 39  163 7 246 9   169 9 0   0  128 239 218 148 252 185 0
0 103 163 7 245 9   196 9 254 15 128 243 126 135 252 210 228
0 167 163 7 248 9   238 9 0   0  128 250 134 158 3   96  27
0 231 163 7 4   10  102 8 0   0  128 3   213 58  6   190 172


command byte, imu, emg, single trailing

0 163 33 2 23 8 16 8 204 4 0 53 0 54 255 251 250 0   252 255 71 251 227 84  6   211 127  x

0 144 23 2 22 8 18 8 207 4 0 39 0 12 255 245 108 255 232 255 168 223 236 0   242 255 24  1
0 240 23 2 22 8 16 8 207 4 0 51 0 60 255 238 108 255 232 255 168 223 234 0   240 255 22  1
0 80  24 2 23 8 15 8 206 4 0 51 0 49 255 250 128 255 208 255 176 223 234 0   240 255 22  1
0 176 24 2 23 8 15 8 206 4 0 31 0 45 0   12  128 255 208 255 176 223 234 0   240 255 22  1
0 16  25 2 21 8 16 8 206 4 0 38 0 56 255 246 128 255 208 255 176 223 237 0   244 255 22  1
0 112 25 2 22 8 15 8 206 4 0 51 0 38 0   3   116 255 216 255 168 223 237 0   244 255 22  1
p ttt tt a aa a aa a aaa g g gg g gg ggg mmm mmm mmm mmm mmm mmm ch1 ch1 ch1 ch2 ch2 ch2 ?

0 111 22 3 44 5 66 7 888 9 1011 1213 14  15  16  17  18  19  20  21  22  23  24  25  26  27


p    ts0 ts1        accel          gyro          mag               ch1        emg    ch2
(0,) 58 179   3842 3080 52488   4 37 65318  -440 -8 -56    -50.69361973130144 -189.06838843048277
(0,) 186 179  4354 2824 53256   4 19 53      840 -8 -56    -50.80439855310105 -179.61547866846863
(0,) 26 180   4098 3336 53256   4 36 65324  -5564 -20 -52  -50.80439855310105 -179.61547866846863
(0,) 122 180  4098 3080 52744   4 32 65331  -3516 -20 -52  -50.80439855310105 -179.61547866846863
(0,) 250 180  4098 3080 52488   4 39 65321  -6588 -20 -52  -50.699774110290306 -187.4930597336761

Just imu:
(0,) 215 113  3843 3080 52232   4 53 65329  -6964 130 -248
(0,) 23 114   3843 3080 52488   4 57 65334  -2866 135 -256
(0,) 119 114  3843 3080 52488   4 51 65328  -3122 135 -256
(0,) 183 114  3587 3080 52232   4 46 65324  -1074 135 -256
(0,) 247 114  3587 3080 53000   4 53 65324  -1330 135 -256



0 118 243 3 225 9 148 9 0   0  128 6 124 251 241 209 145
0 182 243 3 218 9 133 9 0   0  128 5 41  123 237 67  60
0 246 243 3 224 9 144 9 0   0  128 1 191 178 237 19  101
0 54  244 3 223 9 155 9 0   0  128 1 98  64  241 67  54
0 118 244 3 218 9 141 9 0   0  128 2 188 185 253 138 225
0 182 244 3 222 9 140 9 0   0  128 3 124 91  1   91  159
0 246 244 3 214 9 119 9 0   0  128 5 234 146 0   43  75
0 54  245 3 224 9 176 9 1   0  128 7 193 56  0   125 200
0 118 245 3 226 9 101 9 255 15 128 8 45  121 253 142 155
0 182 245 3 221 9 138 9 0   0  128 7 234 188 248 249 197
0 246 245 3 215 9 136 9 0   0  128 7 81  175 245 75  55


29??
0    97  100    5 16 8 9  8 207    4 0 48 0 60 255    235 34 0 57 255 10    1 128 1      109 116 3    202 70
0    193 100    5 17 8 10 8 206    4 0 51 0 35 255    237 34 0 57 255 10    1 128 8      94  68  6    82  151
0    33  101    5 16 8 11 8 206    4 0 39 0 41 255    227 34 0 57 255 10    1 128 246    124 188 6    13  90
0    129 101    5 16 8 12 8 207    4 0 43 0 34 255    233 39 0 58 255 1     1 128 245    69  151 5    85  164
0    225 101    5 17 8 11 8 205    4 0 44 0 42 255    230 39 0 58 255 1     1 128 0      217 125 4    141 52
0    65  102    5 17 8 11 8 205    4 0 41 0 33 255    232 39 0 58 255 1     1 128 6      19  56  3    59  152
0    161 102    5 17 8 11 8 205    4 0 42 0 40 255    243 26 0 50 255 10    1 128 2      96  150 3    82  198
0    1   103    5 18 8 9  8 206    4 0 46 0 43 255    232 26 0 50 255 10    1 128 2      130 251 4    83  131
0    97  103    5 17 8 12 8 206    4 0 62 0 46 255    232 26 0 50 255 10    1 128 9      218 231 7    91  62
0    193 103    5 15 8 10 8 206    4 0 45 0 29 255    241 29 0 53 255 7     1 128 243    230 91  5    147 115
0    33  104    5 15 8 10 8 204    4 0 50 0 40 255    229 29 0 53 255 7     1 128 246    126 123 5    44  126
p    tt  ttt    a aa a aa a aaa    g g gg g gg ggg    mmm mm m mm mmm mm    x  c1 c1     c1  c2  c2   c2  ?? 








    'b': signed char (8-bit integer, -128 to 127)
    'B': unsigned char (8-bit integer, 0 to 255)
    'h': short (16-bit integer, -32,768 to 32,767)
    'H': unsigned short (16-bit integer, 0 to 65,535)
    'i' or 'l': int or long (32-bit integer, -2,147,483,648 to 2,147,483,647)
    'I' or 'L': unsigned int or unsigned long (32-bit integer, 0 to 4,294,967,295)
    'q': long long (64-bit integer, -9,223,372,036,854,775,808 to 9,223,372,036,854,775,807)
    'Q': unsigned long long (64-bit integer, 0 to 18,446,744,073,709,551,615)
    'f': float (32-bit floating point)
    'd': double (64-bit floating point)
"""