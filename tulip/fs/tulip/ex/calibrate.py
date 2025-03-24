# calibrate.py
# find touch delta
import time, tulip

(w,h) = tulip.screen_size()
test_pos = [
    [w/2,h/2],
    [20, h/2],
    [w-10, h/2],
    [w/2,20],
    [w/2,h-20],
    [w/2-20, h/2],
    [w/2+20, h/2]
    ]

got = [-1. -1]
deltas = []
def touch_cb(x):
    global got
    t = tulip.touch()
    if(x==0): # down
        got = [t[0], t[1]]

tulip.touch_callback(touch_cb)

#clear

print("Tap the middle of the pink circles.")
for (x,y) in test_pos:
    x = int(x)
    y = int(y)
    tulip.bg_clear()
    time.sleep_ms(500)
    got = [-1, -1]
    tulip.bg_circle(x,y,20, 194, 1)
    while(got[0] < 0):
        time.sleep_ms(50)
    deltas.append( (x- got[0], y-got[1]) )

x_mean = 0
y_mean = 0
for (x,y) in deltas:
    x_mean += x
    y_mean += y


old_delta = tulip.touch_delta()
x_mean = int(float(x_mean) / float(len(deltas)))
y_mean = int(float(y_mean) / float(len(deltas)))

# TODO: compute this from the deltas
y_scale = old_delta[2]

print("New computed delta is [%d, %d, %f]. It was [%d, %d, %f]. " %(x_mean, y_mean, y_scale, old_delta[0], old_delta[1], old_delta[2]))
if(tulip.prompt("Set it?")):
    tulip.touch_delta(x_mean, y_mean, y_scale)
    if(tulip.prompt("Set. Add calibration to boot.py?")):
        tulip.add_to_bootpy("tulip.touch_delta(%d,%d,%f)" % (x_mean, y_mean, y_scale))
        print("Added.")

tulip.bg_clear()
tulip.touch_callback()
