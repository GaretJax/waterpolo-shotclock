#!/usr/bin/env python3

import time
import math
import enum
import threading
from datetime import timedelta

import attr

import serial
import digitalio
import board

from RPi import GPIO
from adafruit_character_lcd.character_lcd import Character_LCD_RGB


def seconds_or_timedelta(value):
    if isinstance(value, timedelta):
        return value
    else:
        return timedelta(seconds=value)


@attr.s()
class Clock:
    @property
    def current_tick(self):
        return time.time()

    def timesince(self, tick):
        return timedelta(seconds=self.current_tick - tick)


@attr.s(repr=False)
class Timer:
    clock = attr.ib()
    initial = attr.ib(converter=seconds_or_timedelta)

    _started_at = attr.ib(init=False, default=None)
    _elapsed = attr.ib(init=False, default=timedelta())
    _cb_handle = attr.ib(init=False)
    _waiter = attr.ib(init=False, default=None)

    def __repr__(self):
        name = self.__class__.__name__
        remaining = self.remaining.total_seconds()
        initial = self.initial.total_seconds()
        indicator = "*" if self.is_running else " "
        return f"{name}({remaining:.3f} / {initial:.3f} {indicator})"

    @property
    def remaining(self):
        return self.initial - self.elapsed

    @property
    def elapsed(self):
        if self._started_at:
            return self._elapsed + self.clock.timesince(self._started_at)
        else:
            return self._elapsed

    @property
    def is_running(self):
        return self._started_at is not None

    @property
    def is_paused(self):
        return not self.is_running

    @property
    def is_elapsed(self):
        return self.elapsed >= self.initial

    def run(self):
        if self.is_running:
            return
        self._set_tick()

    def pause(self):
        if self.is_paused:
            return
        self._elapsed += self.clock.timesince(self._started_at)
        self._unset_tick()

    def toggle(self):
        if self.is_running:
            self.pause()
        else:
            self.run()

    def reset(self, initial=None):
        if initial:
            self.initial = seconds_or_timedelta(initial)
        self._elapsed = timedelta()
        if self.is_running:
            self._set_tick()

    def correct(self, value):
        delta = seconds_or_timedelta(value)
        assert self.is_paused
        self._elapsed -= delta

    def _set_tick(self):
        self._started_at = self.clock.current_tick

    def _unset_tick(self):
        self._started_at = None


class PWMOut:
    def __init__(self, pin):
        GPIO.setup(pin.id, GPIO.OUT)
        self.out = GPIO.PWM(pin.id, 500)
        self._duty_cycle = 2 ** 15
        self.out.start(100)

    @property
    def duty_cycle(self):
        return self._duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value):
        self._duty_cyle = value
        self.out.ChangeDutyCycle(value / (2 ** 16) * 100)


@attr.s
class ShotclockDisplay:
    CHARS = {
        False: 0b01101001,
        True: 0b00110101,
        " ": 0b01101001,
        "0": 0b00110011,
        "1": 0b00110101,
        "2": 0b00110110,
        "3": 0b00111001,
        "4": 0b00111010,
        "5": 0b01101100,
        "6": 0b10010110,
        "7": 0b10011001,
        "8": 0b10011010,
        "9": 0b10011100,
    }

    _serial = attr.ib()
    _address = attr.ib()
    _clock = attr.ib()
    _update_interval = attr.ib(default=timedelta(seconds=1))
    _initial = attr.ib(default=0)
    _alarm = attr.ib(default=False, init=False)
    _is_paused = attr.ib(default=True, init=False)
    _display_on = attr.ib(default=False, init=False)

    def __attrs_post_init__(self):
        self._value = self._initial
        self._last_write = self._clock.current_tick

    def turn_on(self):
        self._display_on = True
        self._write()

    def turn_off(self):
        self._display_on = False
        self._write()

    def toggle_display(self):
        self._display_on = not self._display_on
        self._write()

    def set(self, value):
        value = int(value)
        if (
            self._value != value
            or self._clock.timesince(self._last_write) > self._update_interval
        ):
            self._value = value
            self._write()

    def _write(self):
        self._serial.write(
            self.frame(
                self._address,
                self._value,
                alarm=self._alarm,
                is_paused=self._is_paused,
                display_on=self._display_on,
            )
        )
        self._last_write = clock.current_tick

    @classmethod
    def frame(
        cls,
        address,
        value,
        *,
        format="2d",
        alarm=False,
        is_paused=False,
        display_on=True,
    ):
        fmt = f"{value:{format}}"
        return bytes(
            [
                address,
                cls.CHARS[fmt[0]],
                cls.CHARS[fmt[1]],
                cls.CHARS[bool(alarm)],
                cls.CHARS[bool(is_paused)],
                cls.CHARS[bool(display_on)],
            ]
        )


@attr.s
class Corrector:
    timer = attr.ib()
    step = attr.ib(default=-1)

    def __call__(self):
        if not timer.is_running:
            self.timer.correct(self.step)


@attr.s
class Resetter:
    timer = attr.ib()
    value = attr.ib()

    def __call__(self):
        self.timer.reset(self.value)


@attr.s
class Button:
    pin = attr.ib()
    callback = attr.ib()
    bounce_period = attr.ib(default=120)
    _last_event = attr.ib(init=False, factory=lambda: int(time.time() * 1000))
    _pressed = attr.ib(init=False, default=False)
    _pressed2 = attr.ib(init=False, default=False)

    def __attrs_post_init__(self):
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(
            self.pin,
            GPIO.BOTH,
            callback=self.handle,
            bouncetime=self.bounce_period,
        )

    def handle(self, channel):
        ts = int(time.time() * 1000)
        self._pressed2 = not self._pressed2
        if ts - self._last_event > self.bounce_period:
            if self._pressed != self._pressed2 and not self._pressed2:
                self._last_event = ts
                self.callback()
            self._pressed = self._pressed2


@attr.s
class Relay:
    pin = attr.ib()

    def __attrs_post_init__(self):
        GPIO.setup(self.pin, GPIO.OUT)
        self.deactivate()

    def activate(self):
        GPIO.output(self.pin, True)

    def deactivate(self):
        GPIO.output(self.pin, False)


@attr.s
class AttributesStore:
    _values = attr.ib(factory=dict)
    _lock = attr.ib(factory=threading.Condition)

    def set(self, attr, value):
        with self._lock:
            if self._values.get(attr) != value or attr is None:
                self._values[attr] = value
                self._lock.notify()

    def get(self):
        with self._lock:
            while not self._values:
                self._lock.wait()
            self._values, values = {}, self._values
        return values


@attr.s
class Symbol:
    index = attr.ib()
    spec = attr.ib()


class SymbolsSet(enum.Enum):
    @classmethod
    def register(cls, lcd):
        for symbol in cls:
            lcd.create_char(symbol.value.index, symbol.value.spec)

    def __str__(self):
        return chr(self.value.index)


def diff(new, old):
    in_diff = False
    seq = ""
    for i, (c1, c2) in enumerate(zip(new, old)):
        if c1 != c2:
            if not in_diff:
                cur = i
                in_diff = True
            seq += c1
        elif in_diff:
            in_diff = False
            yield cur, seq
            seq = []

    if in_diff:
        yield cur, seq


def lcd_updater(lcd, store):
    values = {}
    running = True
    while running:
        for attr, value in store.get().items():
            if attr is None:
                running = False
                break
            old_value = values.get(attr)
            if value != old_value:
                values[attr] = value
                if attr == "message" and old_value:
                    # Set individual chars instead of the whole display
                    # to optimize the time required for the update
                    # (down to <10ms from an initial ~120ms)
                    lines_new = value.split("\n")
                    lines_old = old_value.split("\n")
                    for row, (ln, lo) in enumerate(zip(lines_new, lines_old)):
                        for col, seq in diff(ln, lo):
                            lcd.row = row
                            lcd.column = col
                            lcd.message = seq
                else:
                    setattr(lcd, attr, value)
    lcd.clear()
    lcd.display = False


# Constants
SERIAL_DEVICE = "/dev/ttyS0"
SERIAL_SPEED = 9600
ADDR = 0xEE

CONSOLE_STOP = 233
CONSOLE_START = 239
CONSOLE_TICK = 232
CONSOLE_TICK_TIMEOUT = timedelta(seconds=1)

MINIMUM_UI_UPDATE_INTERVAL = timedelta(milliseconds=20)

PIN_RELAIS = 27

BUTTON_1 = 6
BUTTON_2 = 13
BUTTON_3 = 19
BUTTON_4 = 26

SHOTCLOCK_SECONDS = 30, 20


class Symbols(SymbolsSet):
    PLAY = Symbol(0, [0x0, 0x8, 0xC, 0xE, 0xC, 0x8, 0x0, 0x0])
    PAUSE = Symbol(1, [0x0, 0x1B, 0x1B, 0x1B, 0x1B, 0x1B, 0x0, 0x0])
    RESET_BACKWARDS = Symbol(2, [0x1C, 0x2, 0x1, 0x5, 0x9, 0x1E, 0x8, 0x4])
    RESET_FORWARDS = Symbol(3, [0x7, 0x8, 0x10, 0x14, 0x12, 0xF, 0x2, 0x4])
    MINUS_ONE = Symbol(4, [0x0, 0x1, 0x3, 0x19, 0x1, 0x1, 0x0, 0x0])
    IN_SYNC = Symbol(5, [0x18, 0x18, 0, 0, 0, 0x0, 0x0, 0x0])


LEGEND_ELAPSED = f"{Symbols.RESET_FORWARDS}{SHOTCLOCK_SECONDS[1]}          {SHOTCLOCK_SECONDS[0]}{Symbols.RESET_BACKWARDS}"
LEGEND_RUNNING = f"{Symbols.RESET_FORWARDS}{SHOTCLOCK_SECONDS[1]}   {Symbols.PAUSE}      {SHOTCLOCK_SECONDS[0]}{Symbols.RESET_BACKWARDS}"
LEGEND_STOPPED = f"{Symbols.RESET_FORWARDS}{SHOTCLOCK_SECONDS[1]}   {Symbols.PLAY}  {Symbols.MINUS_ONE}   {SHOTCLOCK_SECONDS[0]}{Symbols.RESET_BACKWARDS}"
COLOR_ELAPSED = [100, 0, 0]
COLOR_RUNNING = [100, 100, 100]
COLOR_STOPPED = [0, 100, 0]
COLOR_OFF = [0, 0, 0]


# Initialization
GPIO.setmode(GPIO.BCM)

clock = Clock()

bus = serial.Serial(SERIAL_DEVICE, SERIAL_SPEED, timeout=0)

display = ShotclockDisplay(bus, ADDR, clock)
display.turn_on()

relay = Relay(PIN_RELAIS)

timer = Timer(clock, SHOTCLOCK_SECONDS[0])

# Buttons
Button(BUTTON_1, Resetter(timer, SHOTCLOCK_SECONDS[1]), bounce_period=180)
Button(BUTTON_2, timer.toggle)
Button(BUTTON_3, Corrector(timer))
Button(BUTTON_4, Resetter(timer, SHOTCLOCK_SECONDS[0]))

# LCD updater
cols = 16
rows = 2

lcd_rs = digitalio.DigitalInOut(board.D17)
lcd_en = digitalio.DigitalInOut(board.D18)
lcd_d4 = digitalio.DigitalInOut(board.D23)
lcd_d5 = digitalio.DigitalInOut(board.D24)
lcd_d6 = digitalio.DigitalInOut(board.D7)
lcd_d7 = digitalio.DigitalInOut(board.D12)

red = PWMOut(board.D16)
green = PWMOut(board.D20)
blue = PWMOut(board.D21)

lcd = Character_LCD_RGB(
    lcd_rs,
    lcd_en,
    lcd_d4,
    lcd_d5,
    lcd_d6,
    lcd_d7,
    cols,
    rows,
    red,
    green,
    blue,
)
Symbols.register(lcd)
store = AttributesStore()
store.set("color", COLOR_STOPPED)
t = threading.Thread(target=lcd_updater, kwargs={"lcd": lcd, "store": store})
t.start()


# Business logic
try:
    print("Shotclock ready!")

    last_console_tick = clock.current_tick
    last_update = clock.current_tick

    while True:
        tick_seen = False
        for c in bus.readall():
            if c == CONSOLE_STOP:
                timer.pause()
            elif c == CONSOLE_START:
                timer.run()
            elif c == CONSOLE_TICK:
                last_console_tick = clock.current_tick
                tick_seen = True
            else:
                timer.toggle()

        in_sync = clock.timesince(last_console_tick) < CONSOLE_TICK_TIMEOUT
        ts_until_ui_update = (
            MINIMUM_UI_UPDATE_INTERVAL - clock.timesince(last_update)
        ).total_seconds()

        if ts_until_ui_update > 0:
            if not in_sync:
                time.sleep(ts_until_ui_update)
            continue

        if in_sync and not tick_seen:
            time.sleep(0.001)
            continue

        if timer.is_elapsed:
            remaining = 0
            color = COLOR_ELAPSED
            legend = LEGEND_ELAPSED

            timer.pause()
            relay.activate()
        else:
            remaining = (
                int(math.ceil(timer.remaining.total_seconds() * 10)) / 10
            )

            if timer.is_running:
                legend = LEGEND_RUNNING
                color = COLOR_RUNNING
            else:
                legend = LEGEND_STOPPED
                color = COLOR_STOPPED

            relay.deactivate()

        sync_indicator = Symbols.IN_SYNC if in_sync else " "
        last_update = clock.current_tick

        # Update UI
        display.set(math.ceil(remaining))
        store.set("color", color)
        store.set(
            "message", f'{sync_indicator}     {remaining:>4.1f}"   \n{legend}'
        )
except KeyboardInterrupt:
    print("SIGINT received, exiting...")
finally:
    print("Cleaning up...")
    # Cleanup
    store.set(None, True)
    t.join()
    GPIO.cleanup()
    print("All done!")
