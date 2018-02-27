##
## This file is part of the libsigrokdecode project.
##
## I2C-Decoder: Copyright (C) 2010-2016 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2018 Laserlicht
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd

class SamplerateError(Exception):
    pass

class Decoder(srd.Decoder):
    api_version = 3
    id = 'mm2'
    name = 'MM2'
    longname = 'Märklin Motorola 2'
    desc = 'Decoder for the Märklin Motorola model railway protocol.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['mm2']
    channels = (
        {'id': 'data', 'name': 'Data', 'desc': 'Data line'},
    )
    options = (
        {'id': 'polarity', 'desc': 'Polarity', 'default': 'active-high',
            'values': ('active-low', 'active-high')},
    )
    annotations = (
        ('timing', 'Timing'),
        ('bits', 'Bits'),
        ('trits', 'Trits'),
        ('decoded', 'Decoded'),
        ('decodedmm2', 'Decoded MM2'),
        ('packagenr', 'Package-Nr'),
    )
    annotation_rows = (
         ('timing', 'Timing', (0,)),
         ('bits', 'Bits', (1,)),
         ('trits', 'Trits', (2,)),
         ('decoded', 'Decoded', (3,)),
         ('decodedmm2', 'Decoded MM2', (4,)),
         ('packagenr', 'Package-Nr', (5,)),
    )
    binary = (
        ('raw', 'RAW file'),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.ss_block = self.es_block = None

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_binary = self.register(srd.OUTPUT_BINARY)

    def putx(self, data):
        self.put(self.ss_block, self.es_block, self.out_ann, data)

    def putp(self, period_t):
        # Adjust granularity.
        if period_t == 0 or period_t >= 1:
            period_s = '%.1f s' % (period_t)
        elif period_t <= 1e-12:
            period_s = '%.1f fs' % (period_t * 1e15)
        elif period_t <= 1e-9:
            period_s = '%.1f ps' % (period_t * 1e12)
        elif period_t <= 1e-6:
            period_s = '%.1f ns' % (period_t * 1e9)
        elif period_t <= 1e-3:
            period_s = '%.1f μs' % (period_t * 1e6)
        else:
            period_s = '%.1f ms' % (period_t * 1e3)

        self.put(self.ss_block, self.es_block, self.out_ann, [0, [period_s]])

    def putb(self, data):
        self.put(self.ss_block, self.es_block, self.out_binary, data)

    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')


        if self.options['polarity'] == 'active-low':
            R_TRIG = 'f'
            F_TRIG = 'r'
        else:
            R_TRIG = 'r'
            F_TRIG = 'f'
        first = True
        lastSignal = -1
        
        while True:
            pause_time = 0.0
            
            #wait for packet begin
            while pause_time < 0.0005:                             #4,2ms / 2,1ms bzw. 3*416us / 3*208us
                start_samplenum = self.samplenum
                self.wait({0: R_TRIG})                            #rising edge
                stop_samplenum = self.samplenum
                pause_time = float((stop_samplenum - start_samplenum) / self.samplerate)
            
            if first == True:
                first = False
                self.first_samplenum = self.samplenum
            
            samples = []
            bits = []
            trits = []
            
            #get data (timepoints)
            for i in range(0, 18*2):
                if i == 0:
                    samples.append(self.samplenum)
                else:
                    #wait for triggers and save points
                    self.wait({0: 'e'})
                    samples.append(self.samplenum)
                    
            #analyze if packet in size
            if ((samples[-1] - samples[0]) / self.samplerate > 0.0013 and (samples[-1] - samples[0]) / self.samplerate < 0.0042 and \
                ((samples[2] - samples[0]) / self.samplerate < 0.000125 or (samples[2] - samples[0]) / self.samplerate > 0.000175)):  #3.74ms / 1.87ms    #MFX herausfiltern ~100us / ~200us  
            
                isMagnet = (samples[-1] - samples[0]) / self.samplerate < 0.0025
                
                #analyze
                for i in range(0, 18*2-1):
                    #write timings
                    self.ss_block = samples[i]
                    self.es_block = samples[i+1]
                    period_t = float((samples[i+1] - samples[i]) / self.samplerate)
                    self.putp(period_t)
                    
                for i in range(0, 18*2):
                    #write bits
                    if i % 2 == 1:
                        period = samples[2] - samples[0]
                        self.ss_block = samples[i-1]
                        self.es_block = samples[i-1] + period
                        bit = 1 if ((samples[i]-samples[i-1])/period>0.5) else 0
                        bits.append(bit)
                        self.putx([1, ['%s' % bit]])
                        
                    #write trits
                    if i % 4 == 3:
                        period = samples[4] - samples[0]
                        self.ss_block = samples[i-3]
                        self.es_block = samples[i-3] + period
                        bitnr = int(i/2)-1
                        trit = 1 if (bits[bitnr]==1) and (bits[bitnr+1]==1) else (0 if (bits[bitnr]==0) and (bits[bitnr+1]==0) else 2)
                        trits.append(trit)
                        self.putx([2, ['%s' % trit]])
                        
                        #write decoded data
                        if int((bitnr+1)/2) == 3:
                            address = trits[3] * 3**3 + trits[2] * 3**2 + trits[1] * 3**1 + trits[0]
                            self.ss_block = samples[0]
                            self.es_block = samples[4*4]
                            self.putx([3, ['Adress: %s' % address]])
                            
                        if isMagnet == True:
                            if int((bitnr+1)/2) == 4:
                                self.ss_block = samples[4*4]
                                self.es_block = samples[4*4+4]
                                self.putx([3, ['0']])
                                
                                #todo more bits Magnet
                        else:
                            if int((bitnr+1)/2) == 4:
                                self.ss_block = samples[4*4]
                                self.es_block = samples[4*4+4]
                                self.putx([3, ['Function: %s' % trits[4]]])
                            if int((bitnr+1)/2) == 8:
                                self.ss_block = samples[4*4+4]
                                self.es_block = samples[4*8] + period
                                
                                step = bits[10] + bits[12] * 2**1 + bits[14] * 2**2 + bits[16] * 2**3
                                if step == 0:
                                    self.putx([3, ['Speed: Stop']])
                                elif step == 1:
                                    self.putx([3, ['Speed: Dir-change']])
                                else:
                                    self.putx([3, ['Speed: %s' % (step-1)]])
                                
                                #todo mm2
                                
                #write packagenr
                self.ss_block = samples[0]
                self.es_block = samples[4*8] + period
                
                pauseTime = (samples[0]-lastSignal) / self.samplerate
                tritTime = (samples[4] - samples[0]) / self.samplerate
                
                if(abs(tritTime*3-pauseTime)<0.0003): #max 300us tolerance
                    self.putx([5, ['Package: 2']])
                else:
                    self.putx([5, ['Package: 1']])
                lastSignal = samples[-1] + (samples[1] - samples[0])
