##
## pd.py — XX SCP sigrok decoder
##

import sigrokdecode as srd
from common.srdhelper import bitpack
from math import floor, ceil

UI = 160e-6
QUARTER_UI = UI / 4
PING_UI = UI * 16
SEP_TOL = 0.2

def parity_ok(parity_type, parity_bit, data, num_data_bits):
    if parity_type == 'zero':
        return parity_bit == 0
    elif parity_type == 'one':
        return parity_bit == 1
    ones = bin(data).count('1') + parity_bit
    if parity_type == 'odd':
        return (ones % 2) == 1
    elif parity_type == 'even':
        return (ones % 2) == 0

class SamplerateError(Exception):
    pass

class Decoder(srd.Decoder):
    api_version = 3
    id       = 'scp'
    name     = 'SCP'
    longname = 'Smart Charge Protocol'
    desc     = 'Smart Charge Protocol'
    license  = 'gplv2+'
    inputs   = ['logic']
    outputs  = ['scp']
    tags     = ['Embedded/Industrial', 'Power']

    channels = (
        {'id': 'rxtx', 'type': 209, 'name': 'D-', 'desc': 'D-'},
    )
    options = (
        {'id': 'num_data_bits', 'desc': 'Data bits', 'default': 8,
            'values': tuple(range(4, 129, 1))},
        {'id': 'parity_check', 'desc': 'Check parity?', 'default': 'yes',
            'values': ('yes', 'no')},
        {'id': 'bit_order', 'desc': 'Bit order', 'default': 'msb-first',
            'values': ('lsb-first', 'msb-first')},
        {'id': 'format', 'desc': 'Data format', 'default': 'hex',
            'values': ('ascii', 'dec', 'hex', 'oct', 'bin')},
        {'id': 'invert', 'desc': 'Invert signal?', 'default': 'no',
            'values': ('yes', 'no')},

    )
    annotations = (
        ('108', 'data',       'data'),
        ('7',   'start',      'start bits'),
        ('6',   'parity-ok',  'parity OK bits'),
        ('0',   'parity-err', 'parity error bits'),
        ('1',   'stop',       'stop bits'),
        ('1000','warnings',   'warnings'),
        ('209', 'data-bits',  'data bits'),
        ('10',  'break',      'break'),
    )
    annotation_rows = (
        ('data',      'RX/TX',    (0, 1, 2, 3, 4)),
        ('data-bits', 'Bits',     (6,)),
        ('warnings',  'Warnings', (5,)),
        ('break',     'break',    (7,)),
    )
    binary = (
        ('rxtx', 'RX/TX dump'),
    )
    idle_state = 'WAIT FOR START BIT'

    def putx(self, data):
        s, halfbit = self.startsample, self.bit_width / 2.0
        self.put(s - floor(halfbit), self.samplenum + ceil(halfbit),
                    self.out_ann, data)

    def putpx(self, data):
        s, halfbit = self.startsample, self.bit_width / 2.0
        self.put(s - floor(halfbit), self.samplenum + ceil(halfbit),
                 self.out_python, data)

    def putg(self, data):
        s, halfbit = self.samplenum, self.bit_width / 2.0
        self.put(s - floor(halfbit), s + ceil(halfbit), self.out_ann, data)

    def putp(self, data):
        s, halfbit = self.samplenum, self.bit_width / 2.0
        self.put(s - floor(halfbit), s + ceil(halfbit), self.out_python, data)

    def putgse(self, ss, es, data):
        self.put(ss, es, self.out_ann, data)

    def putpse(self, ss, es, data):
        self.put(ss, es, self.out_python, data)

    def putbin(self, data):
        s, halfbit = self.startsample, self.bit_width / 2.0
        self.put(s - floor(halfbit), self.samplenum + ceil(halfbit),
                 self.out_binary, data)

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate    = None
        self.samplenum     = 0
        self.frame_start   = -1
        self.frame_valid   = None
        self.startbit      = -1
        self.cur_data_bit  = 0
        self.datavalue     = 0
        self.paritybit     = -1
        self.stopbit1      = -1
        self.startsample   = -1
        self.state         = 'WAIT FOR START BIT'
        self.databits      = []
        self.start_high_count = 0
        self.last_fall_ss     = -1
        self.last_rise_ss     = -1

    def start(self):
        self.out_python = self.register(srd.OUTPUT_PYTHON)
        self.out_binary = self.register(srd.OUTPUT_BINARY)
        self.out_ann    = self.register(srd.OUTPUT_ANN)
        self.bw = (self.options['num_data_bits'] + 7) // 8

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value
            self.bit_width = float(self.samplerate) * UI

    def get_sample_point(self, bitnum):
        bitpos  = self.frame_start + (self.bit_width - 1) / 2.0
        bitpos += bitnum * self.bit_width
        return bitpos

    def wait_for_start_bit(self, signal):
        pass

    def get_start_bit(self, signal):
        self.startbit     = 0
        self.cur_data_bit = 0
        self.datavalue    = 0
        self.startsample  = -1
        self.putp(['STARTBIT', 0, self.startbit])
        start_ss = int(self.samplenum - self.bit_width)
        start_es = int(self.samplenum)
        self.put(start_ss, start_es, self.out_ann, [1, ['Start', 'S']])
        self.state = 'GET DATA BITS'

    def get_data_bits(self, signal):
        if self.startsample == -1:
            self.startsample = self.samplenum
        self.putg([6, ['%d' % signal]])
        s, halfbit = self.samplenum, int(self.bit_width / 2)
        self.databits.append([signal, s - halfbit, s + halfbit])
        self.cur_data_bit += 1
        if self.cur_data_bit < self.options['num_data_bits']:
            return
        bits = [b[0] for b in self.databits]
        if self.options['bit_order'] == 'msb-first':
            bits.reverse()
        self.datavalue = bitpack(bits)
        self.putpx(['DATA', 0, (self.datavalue, self.databits)])
        self.putx([0, ['@0x%02X' % self.datavalue]])
        b = self.datavalue
        bdata = b.to_bytes(self.bw, byteorder='big')
        self.putbin([0, bdata])
        self.putbin([1, bdata])
        self.databits = []
        self.state = 'GET PARITY BIT'

    def get_parity_bit(self, signal):
        self.paritybit = signal
        if parity_ok('odd', self.paritybit,
                     self.datavalue, self.options['num_data_bits']):
            self.putp(['PARITYBIT', 0, self.paritybit])
            self.putg([2, ['Parity bit', 'Parity', 'P']])
        else:
            self.putp(['PARITY ERROR', 0, (0, 1)])
            self.putg([3, ['Parity error', 'Parity err', 'PE']])
            self.frame_valid = False
        self.state = 'GET STOP BITS'

    def get_stop_bits(self, signal):
        self.putp(['STOPBIT', 0, 1])
        start_ss = int(self.samplenum)
        start_es = int(self.samplenum + ceil(self.bit_width / 4.0))
        self.put(start_ss, start_es, self.out_ann, [1, ['Stop bit', 'Stop', 'T']])

        es = self.samplenum + ceil(self.bit_width / 2.0)
        self.putpse(self.frame_start, es,
                    ['FRAME', 0, (self.datavalue, self.frame_valid)])

        self.state = 'WAIT FOR SEP END'

    def get_wait_cond(self, inv):
        state = self.state
        if state == 'WAIT FOR START BIT':
            return {0: 'e'}
        if state == 'WAIT FOR SEP END':
            return {0: 'f'}
        if state == 'GET START BIT':
            return {'skip': 0}
        elif state == 'GET DATA BITS':
            bitnum = self.cur_data_bit
        elif state == 'GET PARITY BIT':
            bitnum = self.options['num_data_bits']
        elif state == 'GET STOP BITS':
            return {0: 'r'}
        else:
            return {0: 'e'}
        want_num = ceil(self.get_sample_point(bitnum))
        return {'skip': want_num - self.samplenum}

    def inspect_sample(self, signal, inv):
        if inv:
            signal = not signal
        state = self.state
        if state == 'WAIT FOR START BIT':
            self.wait_for_start_bit(signal)
        elif state == 'GET START BIT':
            self.get_start_bit(signal)
        elif state == 'GET DATA BITS':
            self.get_data_bits(signal)
        elif state == 'GET PARITY BIT':
            self.get_parity_bit(signal)
        elif state == 'GET STOP BITS':
            self.get_stop_bits(signal)

    def inspect_edge(self, signal, inv):
        if inv:
            signal = not signal

        if self.state == 'GET DATA BITS' and self.cur_data_bit == 0:
            self.frame_start  = self.samplenum

        if signal == 1:
            self.last_rise_ss = self.samplenum
        else:
            self.last_fall_ss = self.samplenum
            high_dur = (self.samplenum - self.last_rise_ss) / self.samplerate

            if self.state == 'GET STOP BITS' or self.state == 'WAIT FOR START BIT':
                sep_lo = PING_UI * (1 - SEP_TOL)
                sep_hi = PING_UI * (1 + SEP_TOL)
                if sep_lo <= high_dur <= sep_hi:
                    self.frame_start  = self.samplenum
                    self.frame_valid  = True
                    self.cur_data_bit = 0
                    self.datavalue    = 0
                    self.paritybit    = -1
                    self.startsample  = -1
                    self.databits     = []
                    self.state = 'WAIT FOR START BIT'
                    self.putgse(self.last_rise_ss, self.samplenum,
                                [0, ['PING', 'PING', 'PG']])

            if self.state == 'WAIT FOR SEP END':
                sep_lo = QUARTER_UI * (1 - SEP_TOL)
                sep_hi = QUARTER_UI * (1 + SEP_TOL)
                if sep_lo <= high_dur <= sep_hi:
                    self.frame_start  = self.samplenum
                    self.frame_valid  = True
                    self.cur_data_bit = 0
                    self.datavalue = 0
                    self.paritybit = -1
                    self.startsample = -1
                    self.databits = []
                    self.state = 'GET DATA BITS'
                else:
                    self.putgse(self.last_rise_ss, self.samplenum,
                                [5, ['SEP width err', 'SEP err', '!']])
                    self.state = 'WAIT FOR START BIT'
                    self.start_high_count = 0

        if self.state != 'WAIT FOR START BIT':
            return

        if signal == 0 and self.last_rise_ss >= 0:
            high_dur = (self.samplenum - self.last_rise_ss) / self.samplerate
            hi_quarter_lo = QUARTER_UI * (1 - SEP_TOL)
            hi_quarter_hi = QUARTER_UI * (1 + SEP_TOL)

            if hi_quarter_lo <= high_dur <= hi_quarter_hi:
                self.start_high_count += 1
                if self.start_high_count == 2:
                    self.frame_start      = self.samplenum
                    self.frame_valid      = True
                    self.start_high_count = 0
                    self.state            = 'GET START BIT'

    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')

        inv = self.options['invert'] == 'yes'
        cond_data_idx = None
        cond_edge_idx = None

        while True:
            conds = []
            cond_data_idx = len(conds)
            conds.append(self.get_wait_cond(inv))
            cond_edge_idx = len(conds)
            conds.append({0: 'e'})

            (rxtx,) = self.wait(conds)

            if cond_data_idx is not None and (self.matched & (0b1 << cond_data_idx)):
                self.inspect_sample(rxtx, inv)
            if cond_edge_idx is not None and (self.matched & (0b1 << cond_edge_idx)):
                self.inspect_edge(rxtx, inv)
