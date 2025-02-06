# tulip.py

# Bring in all c-defined tulip functions
from _tulip import *
from upysh import cd, pwd
import amy


def sys():
    return root_dir()+"sys/"

import midi

# convert tulip RGB332 pal_idx to 3 rgb 0-255 values
def rgb(px0, wide=False):
    r = px0 & 0xe0;
    g = (px0 << 3) & 0xe0
    b = (px0 << 6) & 0xc0

    # If "wide", carry over the smallest bit to the rest of the bits 
    # This is like setting extra RGB TTL pins to high
    if(wide):
        if(r & 0b00100000): r = r | 0b00011111
        if(g & 0b00100000): g = g | 0b00011111
        if(b & 0b01000000): b = b | 0b00111111
    return (r,g,b)

from ui import *
from editor import edit

# A class for making a game that runs in a UIScreen. Clears and sets up the screen for a game
class UIScreenGame():
    def __init__(self):
        key_scan(1) # enter direct scan mode, keys will not hit the REPL this way
        Sprite.reset()  # resets sprite counter and RAM
        collisions() # resets collision
        amy.reset()

    def quit(self):
        collisions() # resets collision
        Sprite.reset()  # resets sprite counter
        key_scan(0)
        amy.reset()


# A class for making a game. Clears and sets up the screen for a game
class Game():
    def __init__(self, debug=False):
        self.debug = debug
        if(not debug):
            tfb_stop()
        key_scan(1) # enter direct scan mode, keys will not hit the REPL this way
        Sprite.reset()  # resets sprite counter and RAM
        collisions() # resets collision
        amy.reset()
        self.run = 1

    def quit(self):
        self.run = 0
        # Done. Clean up a bit
        frame_callback()
        collisions() # resets collision
        if(not self.debug):
            Sprite.reset()  # resets sprite counter
            gpu_reset()
            tfb_restore()
            tfb_start()
        key_scan(0)

# Class to handle sprites, takes care of memory
class Sprite():
    mem_pointer = 0
    num_sprites = 0
    SPRITE_RAM_BYTES = 32*1024
    SPRITES = 32
    SCREEN_WIDTH, SCREEN_HEIGHT = screen_size()

    def reset():
        Sprite.mem_pointer = 0
        Sprite.num_sprites = 0
        sprite_clear()

    def __init__(self, sprite_id=None, copy_of=None):
        if(sprite_id is None):
            self.sprite_id = Sprite.num_sprites
            Sprite.num_sprites += 1
        else:
            self.sprite_id = sprite_id

        # Lets you init a new sprite but using existing bitmap data (for multiple copies of a sprite on screen)
        if(copy_of is not None):
            self.mem_pos = copy_of.mem_pos
            self.width = copy_of.width
            self.height = copy_of.height
        else:
            self.mem_pos = None
            self.width = None
            self.height = None
        self.x = 0.0
        self.y = 0.0
        self.x_v = 0.0
        self.y_v = 0.0

    def clamp(self):
        # Modifies x,y to be within the visible screen area
        if(self.x < 0):
            self.x = 0
        if(self.x >= Sprite.SCREEN_WIDTH-self.width):
            self.x = Sprite.SCREEN_WIDTH - (self.width + 1)
        if(self.y < 0):
            self.y = 0
        if(self.y >= Sprite.SCREEN_HEIGHT-self.height):
            self.y = Sprite.SCREEN_HEIGHT - (self.height + 1)

    # Load a sprite into Tulip and keep track of memory
    def load(self, filename, width, height):
        if(self.sprite_id >= Sprite.SPRITES):
            raise Exception("No more sprite handles.")
        else:
            if(self.mem_pos is not None):
                sprite_register(self.sprite_id,self.mem_pos, self.width, self.height)
            else:
                self.width = width
                self.height = height
                if((Sprite.mem_pointer + (height*width)) > Sprite.SPRITE_RAM_BYTES):
                    raise Exception("No more sprite RAM. Current pointer %d, you want to add %d" % (Sprite.mem_pointer, (height*width)))
                else:
                    self.mem_pos = Sprite.mem_pointer
                    sprite_png(filename, Sprite.mem_pointer)
                    sprite_register(self.sprite_id,self.mem_pos, self.width, self.height)
                    Sprite.mem_pointer += width*height

    def off(self):
        sprite_off(self.sprite_id)

    def on(self):
        sprite_on(self.sprite_id)

    def moveto(self,x,y):
        # convenience -- set x and y and move there
        self.x = x
        self.y = y
        self.move()

    def move(self):
        sprite_move(self.sprite_id, int(self.x), int(self.y))


# A sprite who can move from the joystick/keyboard
class Player(Sprite):
    def __init__(self, speed=10):
        super().__init__()
        self.x_v = speed
        self.y_v = speed

    def joy_move(self):
        # Move the player based on joyk
        if(joyk() & Joy.RIGHT):
            self.x += self.x_v
        if(joyk() & Joy.LEFT):
            self.x -= self.x_v
        if(joyk() & Joy.UP):
            self.y -= self.y_v
        if(joyk() & Joy.DOWN):
            self.y += self.y_v
        self.clamp() # Make sure we're on screen

class Colors:
    """ ANSI color codes """
    BLACK = "\033[0;30m"
    RED = "\033[0;31m"
    GREEN = "\033[0;32m"
    BROWN = "\033[0;33m"
    BLUE = "\033[0;34m"
    PURPLE = "\033[0;35m"
    CYAN = "\033[0;36m"
    LIGHT_GRAY = "\033[0;37m"
    DARK_GRAY = "\033[1;30m"
    LIGHT_RED = "\033[1;31m"
    LIGHT_GREEN = "\033[1;32m"
    YELLOW = "\033[1;33m"
    LIGHT_BLUE = "\033[1;34m"
    LIGHT_PURPLE = "\033[1;35m"
    LIGHT_CYAN = "\033[1;36m"
    LIGHT_WHITE = "\033[1;37m"
    BG_BLACK = "\033[0;40m"
    BG_RED = "\033[0;41m"
    BG_GREEN = "\033[0;42m"
    BG_BROWN = "\033[0;43m"
    BG_BLUE = "\033[0;44m"
    BG_PURPLE = "\033[0;45m"
    BG_CYAN = "\033[0;46m"
    BG_LIGHT_GRAY = "\033[0;47m"
    BG_DARK_GRAY = "\033[1;40m"
    BG_LIGHT_RED = "\033[1;41m"
    BG_LIGHT_GREEN = "\033[1;42m"
    BG_YELLOW = "\033[1;43m"
    BG_LIGHT_BLUE = "\033[1;44m"
    BG_LIGHT_PURPLE = "\033[1;45m"
    BG_LIGHT_CYAN = "\033[1;46m"
    BG_LIGHT_WHITE = "\033[1;47m"
    BOLD = "\033[1m"
    FAINT = "\033[2m"
    ITALIC = "\033[3m"
    UNDERLINE = "\033[4m"
    FLASH = BLINK = "\033[5m"
    REVERSE = INVERSE = NEGATIVE = "\033[7m"
    STRIKE = CROSSED = "\033[9m"
    RESET = DEFAULT = END = "\033[0m"
    SORTED_HUE = [0,73,146,219,251,178,105,210,242,137,169,201,233,32,64,96,128,160,192,224,228,196,237,164,205,132,232,246,173,100,200,241,168,236,214,141,209,68,136,204,240,245,172,250,177,104,208,244,213,140,249,176,212,248,255,182,109,218,254,145,181,217,253,36,72,108,144,180,216,252,220,184,221,148,185,112,188,222,149,76,152,189,116,156,186,113,153,40,80,120,124,157,84,190,117,44,88,92,121,48,125,52,56,60,223,150,77,154,158,81,85,89,93,4,8,12,16,20,24,28,61,57,126,53,29,122,49,25,94,21,191,118,45,90,62,17,30,58,159,86,13,26,127,54,95,22,63,31,187,114,155,41,82,123,91,50,59,9,18,27,23,55,87,14,119,46,19,51,151,78,83,5,10,15,47,115,42,11,79,6,43,7,183,110,147,37,74,111,75,38,39,1,2,3,35,71,107,34,143,70,67,103,179,106,139,33,66,99,135,175,102,131,171,98,167,163,215,142,211,69,138,207,203,134,199,65,130,195,227,231,235,162,239,166,243,170,97,194,198,226,247,174,101,202,230,129,234,161,206,133,193,225,238,165,197,229]

class Joy:
    # These are the mask bits for the joystick / keyboard
    # TODO: test NES (not just SNES)
    # Use like (tulip.joy & tulip.Joy.SELECT)
    R1 = 2
    L1 = 4
    X = 8
    A = 16
    RIGHT = 32
    LEFT = 64
    DOWN = 128
    UP = 256
    START = 512
    SELECT = 1024
    Y = 2048
    B = 4096

# prompt for y/n and return true if Y
def prompt(prompt):
    if(board()=="WEB"):
        return webnyi()
    print(prompt + " [Yy/Nn]: ", end='')
    r = input()
    if(r=='Y' or r=='y'): 
        return True
    return

# Add a string to the users' boot.py -- used by some utlities
# if only_first_create is set, will only write if the file doesn't exist yet
def add_to_bootpy(s, only_first_create=False):
    first = False
    try:
        bootpy = open(tulip.root_dir()+"user/boot.py","r").read()
    except OSError:
        first = True
        bootpy = "" # file doesn't exist yet
    
    bootpy = bootpy + "\n" + s + "\n"
    if((only_first_create and first) or (not only_first_create)):
        w = open(tulip.root_dir()+'user/boot.py','w')
        w.write(bootpy)
        w.close()

# Wrapper around AMY tempo to store it
amy_bpm = 108
def seq_bpm(bpm=None):
    global amy_bpm
    if bpm is None:
        return amy_bpm
    else:
        amy.send(tempo=bpm)
        amy_bpm = bpm

def seq_ppq(ppq=None):
    if(ppq is not None):
        print("You can no longer set PPQ in Tulip. It's fixed at %d" % (amy.AMY_SEQUENCER_PPQ))
    return amy.AMY_SEQUENCER_PPQ    

def remap():
    if(board()=="WEB"):
        return webnyi()
    print("Type key or key combo you wish to remap: ",end='')
    (_, scan, mod) = key_wait()
    print()
    print("Type ASCII (0-127) code for the symbol you want (see https://ascii-codes.com/): ", end='')
    code = int(input())
    cmd = "tulip.key_remap(%d,%d,%d)" % (scan, mod, code)
    key_remap(scan, mod, code)
    if(prompt("Remapped. Add \"%s\" to your boot.py?" % (cmd))):
        add_to_bootpy(cmd)
        print("All done. Reboot Tulip!")

def version():
    # Returns current tulip version (aka git hash and compiled date)
    me = build_strings()
    return me[2].replace("-", "") + "-" + me[1].replace("-dirty", "")

def free_disk_bytes():
    import os
    st = os.statvfs('.')
    return st[1] * st[3]

def root_dir():
    try:
        import uos

        root_directory = "/"
        if board() == "DESKTOP":
            xdg_data_directory = uos.getenv("HOME") + "/.local/share"
            
            if (exists(xdg_data_directory)):
                root_directory = xdg_data_directory + "/tulipcc/"
            else:
                # fall back to ~/Documents if XDG base directory spec is not supported
                root_directory = uos.getenv("HOME") + "/Documents/tulipcc/"
        elif board() == "WEB":
            root_directory = "/tulip4/"
        return root_directory
    except:
        return "/"

# This is only (fow now) done from macOS .app packages.
# For linux/windows, it's copied on build -- no predefined packages for those (yet)
# For iOS we work in the app folder, so no need to copy
def desktop_copy_sys(dest):
    import os
    if (app_path().endswith(".app")): # running from an app bundle
        print("Copying system folder from app bundle on first run... ")
        tulip_home = app_path()+"/Contents/Resources/tulip_home"
        cmd = "cp -Rf \"%s/\" \"%s\"" % (tulip_home, dest)
        os.system(cmd)


def get_latest_release():
    import json
    from upysh import rm
    tulip.url_save('https://api.github.com/repos/shorepine/tulipcc/releases/latest','releases_temp.json')
    j = json.load(open('releases_temp.json','r'))
    rm('releases_temp.json')
    mine = None
    sys = None
    for i in j['assets']:
        if i['name'] == 'tulip-firmware-%s.bin' % (board()):
            mine = i
        if i['name'] == 'tulip-sys.bin':
            sys = i
    if(mine and sys):
        return (mine['browser_download_url'], mine['updated_at'], sys['browser_download_url'], sys['updated_at'])
    return (None, None, None, None)

def upgrade():
    import time, sys, os
    import tuliprequests as urequests
    try:
        import esp32, machine
        from esp32 import Partition
    except ImportError:
        print("Upgrading only works on Tulip CC for now. Visit tulip.computer to download the latest Tulip Desktop or Tulip Web.")
        return

    if ip() is None:
        print("Need to be on wifi.")
        return

    # Ensures we are on a OTA partition
    ota_partition = Partition(esp32.Partition.RUNNING).get_next_update()
    sys_partition = Partition.find(Partition.TYPE_DATA, label="system")[0]

    if(ota_partition is None or (not ota_partition.info()[4].startswith('ota'))):
        print("You are not using OTA partitions.")
        return
    if((board() == "OTHER") or (board() == "DESKTOP")):
        print("Unidentified board type, can't upgrade.")
        return

    # Checks for a new firmware from Github, asks if you want to upgrade to it
    sec_size = 4096

    (latest_mine_url, latest_mine_date, latest_sys_url, latest_sys_date) = get_latest_release()
    if(latest_mine_url is None):
        print("Could not find firmware and system folder for board %s" % (board()))
        return

    print("For board: %s" % (board()))
    print("Latest firmware: %s" % (latest_mine_date))
    print("Latest /sys    : %s" % (latest_sys_date))
    print()
    do_firmware = False
    do_system = False

    print("Upgrade firmware?       (Y/[N]) ",end='')
    a = input()
    if(a=='Y' or a=='y'):
        do_firmware=True

    print("Upgrade /sys folder?    (Y/[N]) ",end='')
    a = input()
    if(a=='Y' or a=='y'):
        do_system=True
    
    if(do_system):
        try:
            write_size = sys_partition.info()[3]
            print("Flashing /sys folder... ")
            r = urequests.get(latest_sys_url)
            block_c = 0
            for chunk in r.generate(chunk_size=sec_size):
                if len(chunk) < sec_size:
                    print("Last chunk, adding padding of sys folder")
                    chunk = chunk + bytes(b'\xff'*(sec_size - len(chunk)))
                sys_partition.writeblocks(block_c, chunk)
                if(block_c and (block_c % 128 == 0)):
                    print("%d/%d bytes flashed of sys folder" % (block_c*sec_size, write_size))
                block_c = block_c + 1
            r.close()
            print("Write done.")
        except Exception as e:
            sys.print_exception(e)

    if(do_firmware):
        try:
            write_size = ota_partition.info()[3]
            print("Flashing to OTA partition %s " % (ota_partition.info()[4]))
            r = urequests.get(latest_mine_url)
            block_c = 0
            for chunk in r.generate(chunk_size=sec_size):
                if len(chunk) < sec_size:
                    chunk = chunk + bytes(b'\xff'*(sec_size - len(chunk)))
                ota_partition.writeblocks(block_c, chunk)
                if(block_c and (block_c % 128 == 0)):
                    print("%d/%d bytes flashed of firmware" % (block_c*sec_size, write_size))
                block_c = block_c + 1
            print("Write done. Setting boot partition to %s..." % (ota_partition.info()[4]))
            ota_partition.set_boot()
            # Partition.mark_app_valid_cancel_rollback is set in _boot.py. So if Tulip reboots properly, the partition will be marked OK
        except Exception as e:
            sys.print_exception(e)

    if(do_system or do_firmware): 
        print("Rebooting in 5 seconds...")   
        time.sleep(5)
        machine.reset()

# Return Tulip CC r11 or TDECK battery voltage in V
# This is read from a voltage divider on VBAT into an ADC.
# It's not incredibly accurate, but good enough for a "four-bar" battery indicator
# Things should turn off around 3.1V
def battery(n=5):
    if(board()=='TULIP4_R11'):
        pin = 3
    elif(board()=='TDECK'):
        pin = 4
    else:
        return 0

    import time
    from machine import Pin, ADC
    pot = ADC(Pin(pin))
    pot.atten(ADC.ATTN_11DB)
    x = 0
    for i in range(n):
        x = x + pot.read_uv()
        time.sleep(0.1)
    x = ((x / (float(n))) * 2.0) / 1000000.0
    return x

# scans the keyboard. lets you use either
# Z = B, X = A, A = Y, S = X, enter = START, ' = SELECT, Q = L1, W = R1, arrows = DPAD
def joyk():
    jmask = 0
    key_scans = keys()[1:5] # get up to four keys held at once
    for k in key_scans:
        if(k == 79): jmask = jmask | Joy.RIGHT
        if(k == 80): jmask = jmask | Joy.LEFT
        if(k == 82): jmask = jmask | Joy.UP
        if(k == 81): jmask = jmask | Joy.DOWN
        if(k == 29): jmask = jmask | Joy.B
        if(k == 27): jmask = jmask | Joy.A
        if(k == 4):  jmask = jmask | Joy.Y
        if(k == 22): jmask = jmask | Joy.X
        if(k == 40): jmask = jmask | Joy.START
        if(k == 52): jmask = jmask | Joy.SELECT
        if(k == 20): jmask = jmask | Joy.L1
        if(k == 26): jmask = jmask | Joy.R1
    return jmask

def app(switch=None):
    if(switch is None):
        return running_apps
    current_uiscreen().active = False
    running_apps[switch].present()


def reload(module):
    thing = module
    if(type(thing)!=str): # this is a module, convert to a str
        thing = module.__name__
    try:
        exec('del sys.modules["%s"]' % (thing))
    except KeyError:
        pass # it's ok
    if(not is_folder(thing)):
        exec('import %s' % (thing))
    else:
        cd()


def is_folder(fn):
    import os
    try:
        if(os.stat(fn)[0]&0x4000):
            return True
    except:
        pass
    return False

def exists(fn):
    import os
    try:
        r = os.stat(fn)
    except OSError:
        return False
    return True


def download_and_run(name):
    if board()=="WEB":
        import world_web as world
    else:
        import world
    def rt():
        run(name)
    world.download(name,done_cb=rt)
    print("Downloading '%s'... please wait..." % (name))


# reloads, runs and cleans up a Tulip "app"
# Some ways to run things
# (1) run('module') # imports module.py in your cwd
# (2) run('game') # cds into game and imports game.py
# (3) run('screen_module') # cds into, imports and then calls run(screen) on screen_module.py in your cwd
# (4) run('m5joy') # imports m5joy.py from frozen python module
# (5) run('drums') # imports drums.py and run(screen)s it
# (6) run('bunny_bounce') # finds this in /sys/app/bunny_bounce
# (7) run("calibrate") # finds this in /sys/ex/calibrate.py

def run(module_string):
    import sys
    before_run = sys.modules.copy()
    before_run_pwd = pwd()

    # First, if we're already running, don't run again. Causes some problems
    if module_string in running_apps:
        # Switch to it
        app(module_string)
        return

    # Second, see if we need to delete a running import, for reload
    try:
        exec('del sys.modules["%s"]' % (module_string))
    except KeyError:
        pass # ok!

    # Let's find out what this is. In order, we go: cwd, frozen modules, /sys/app, /ex
    if(is_folder(module_string)):
        cd(module_string)
    elif(exists(root_dir()+"sys/ex/"+module_string+".py")):
        cd(root_dir()+"sys/ex")
    elif(is_folder(root_dir()+"sys/ex/"+module_string)):
        cd(root_dir()+"sys/ex/"+module_string)

    try:
        exec('import %s' % (module_string))
    except Exception as e:
        print("Problem running %s: "  %(module_string))
        sys.print_exception(e)
        cd(before_run_pwd)
        return


    # Now run it. Check for a run() def
    try:
        screen = None
        # Is this a switchable app? 
        actual_module = sys.modules[module_string]
        if(hasattr(actual_module, 'run')):
            # Make the app screen
            screen = tulip.UIScreen(module_string, bg_color=0)
            screen.app_dir = pwd()
            # Run the app
            try:
                actual_module.run(screen)
            except Exception as e:
                print("Error running %s: " % (module_string))
                sys.print_exception(e)
                screen.screen_quit_callback(None)

        else:
            # This was a package but not an app, and we're done. so clean up.
            # Delete the modules we imported
            for imported_module in sys.modules.keys():
                if imported_module not in before_run:
                    exec('del sys.modules["%s"]' % (imported_module))
            repl_screen.present()

        # Go back to where you were
        cd(before_run_pwd)


    except Exception as e:
        print("Error running %s:"% (module_string))
        sys.print_exception(e)
        # Clean up the screen
        if(screen): screen.screen_quit_callback(None)

def url_save(url, filename, mode="wb", headers={"User-Agent":"TulipCC/4.0"}):
    if(board()=="WEB"):
        import world_web, js
        def next(x):
            r = open(filename, mode)
            r.write(x)
            r.close()
            return "OK"
        return world_web.grab_bytes_direct(url, headers=headers).then(lambda x: next(x))
    else:
        import tuliprequests as urequests
        d = urequests.get(url, headers = headers).save(filename,mode)
        return d

def url_get(url, headers={"User-Agent":"TulipCC/4.0"}):
    if(board()=="WEB"):
        import world_web
        return world_web.grab_bytes_direct(url, headers=headers)
    else:
        import tuliprequests as urequests
        c = urequests.get(url, headers = headers)
        return c


def url_put(url, filename, headers={"User-Agent":"TulipCC/4.0"}):
    if(board()=="WEB"):
        return webnyi()
    import os
    import tuliprequests as urequests
    filesize = os.stat(filename)[6]
    f = open(filename, 'rb')
    put_response = urequests.put(
        url,
        headers={
            "User-Agent": headers['User-Agent'],
            "Tulip-Filename": filename,
            "Content-Type": "application/octet-stream",
        },
        data=f,
    )
    f.close()

def screenshot(filename=None, x=-1, y=-1, w=-1, h=-1):
    if(board()=="WEB"):
        import world_web as world
    else:
        import world
    from upysh import rm
    if(filename is not None):
        int_screenshot(filename,x,y,w,h)
        return None
    if(ip() is not None):
        int_screenshot("screenshot.png",x,y,w,h)
        world.upload("screenshot.png", 'Tulip Screenshot')
    else:
        print("Need wi-fi on")


def ansi_fg(pal_idx):
    # ESC[38;5;{ID}m
    return("\033[38;5;%dm" % (pal_idx))

def ansi_bg(pal_idx):
    # ESC[48;5;{ID}m
    return("\033[48;5;%dm" % (pal_idx))

# convert 0-255 r,g,b values to a tulip pal_idx
def color(r,g,b):
    ret = 0;
    ret |= (r&0xe0);
    ret |= (g&0xe0) >> 3
    ret |= (b&0xc0) >> 6
    return (ret & 0xff)


def ip():
    try:
        import network
    except ImportError:
        return "127.0.0.1" # we are on local and it's ok
    sta_if = network.WLAN(network.STA_IF)
    if(sta_if.isconnected()):
        ipv4 = sta_if.ifconfig()[0]
        set_quartet(int(ipv4.split('.')[3]))
        return sta_if.ifconfig()[0]
    else:
        return None

def set_time():
    import ntptime
    if(ip() is None):
        print("Need to be on wifi")
        return
    ntptime.settime()


def wifi(ssid, passwd, wait_timeout=10):
    import network, time
    sta_if = network.WLAN(network.STA_IF)
    sta_if.active(True)
    sta_if.connect(ssid, passwd)
    sleep = 0
    while((sta_if.isconnected() is False) and sleep < wait_timeout):
        sleep = sleep + 1
        time.sleep(1)
    return ip()


def tar_create(directory):
    import utarfile
    t = utarfile.TarFile(directory+".tar", 'w')
    t.add(directory)
    t.close()

def tar_extract(file_name, show_progress=True):
    import os
    import utarfile
    tar = utarfile.TarFile(file_name, 'r')
    if(show_progress): print("extracting", file_name)
    for i in tar:
        if i.type == utarfile.DIRTYPE:
            if i.name != './':
                try:
                    os.mkdir(i.name.strip('/'))
                except OSError as error:
                    if(show_progress): print("Warning: directory",i.name,"already exists")
            if(show_progress): print("making:", i.name)
        else:
            if i.name.startswith(".") or "._" in i.name:
                if(show_progress): print("ignoring", i.name)
            else:
                if(show_progress): print("extracting:", i.name)
                sub_file = tar.extractfile(i)
                data = sub_file.read()
                try:
                    with open(i.name, "wb") as dest:
                        dest.write(data)
                        dest.close()
                except OSError as error:
                    if(show_progress): print("borked on:", i.name)
