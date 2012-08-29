/*

Translation of C version.  Original C version contains the following
license comment:

   A C-program for MT19937, with initialization improved 2002/1/26.
   Coded by Takuji Nishimura and Makoto Matsumoto.

   Before using, initialize the state by using init_genrand(seed)
   or init_by_array(init_key, key_length).

   Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

     3. The names of its contributors may not be used to endorse or promote
        products derived from this software without specific prior written
        permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


   Any feedback is very welcome.
   http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/emt.html
   email: m-mat @ math.sci.hiroshima-u.ac.jp (remove space)
*/
package edu.unc.robotics.prrts.util;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.Random;

/**
 * Java translation of Mersenne Twister psuedo-random number generator.
 */
public class MersenneTwister extends Random {

    /** Number of integers to generate at a time */
    private static final int N = 624;
    private static final int M = 397;
    private static final int UPPER_MASK = 0x80000000;
    private static final int LOWER_MASK = 0x7fffffff;

    private static final int[] MAG01 = { 0, 0x9908b0df };

    private int[] _state;
    private int _index;

    public MersenneTwister() {
//        // Generate a seed using the default implementation
//        // of next(32).  Java's standard PNRG has builtin
//        // seeding with a "uniquifier" to make multiple instances
//        // different.
//
//        int[] seed = new int[N];
//        for (int i=N ; --i >= 0 ; ) {
//            seed[i] = super.next(32);
//        }
//        setSeed(seed);
    }

    public MersenneTwister(int seed) {
        setSeed(seed);
    }

    public MersenneTwister(int[] seed) {
        setSeed(seed);
    }

    public MersenneTwister(byte[] seed) {
        setSeed(seed);
    }

    /**
     * Translation of init_genrand
     *
     * @param seed the seed
     */
    @Override
    public void setSeed(long seed) {
        _state = new int[N];
        _state[0] = (int)seed;

        for (_index=1 ; _index < N ; _index++) {
            _state[_index] = 1812433253
                * (_state[_index - 1] ^ (_state[_index-1] >>> 30))
                +  _index;
        }
    }

    /**
     * Translation of init_by_array.
     *
     * @param seed the seed
     */
    public void setSeed(int[] seed) {
        setSeed(19650218);
        final int seedLength = seed.length;
        int i = 1;
        int j = 0;

        for (int k = Math.max(N, seedLength) ; k >= 0 ; --k) {
            _state[i] = (_state[i] ^ ((_state[i-1] ^ (_state[i-1] >>> 30)) * 1664525))
                + seed[j] + j; // non linear

            if (++i >= N) {
                _state[0] = _state[N-1];
                i=1;
            }

            if (++j >= seedLength) {
                j=0;
            }
        }

        for (int k = N ; --k >= 0 ; ) {
            _state[i] = (_state[i] ^ ((_state[i-1] ^ (_state[i-1] >>> 30)) * 1566083941))
                - i; // non linear

            if (++i >= N) {
                _state[0] = _state[N-1];
                i=1;
            }
        }

        _state[0] = UPPER_MASK; // MSB is 1; assuring non-zero initial array
    }

    private static int[] bytesToInts(byte[] bytes) {
        IntBuffer intBuffer = ByteBuffer.wrap(bytes).asIntBuffer();
        int[] ints = new int[intBuffer.remaining()];
        intBuffer.get(ints);
        return ints;
    }

    public void setSeed(byte[] seed) {
        setSeed(bytesToInts(seed));
    }

    @Override
    protected int next(int bits) {
        int y;

        if (_index >= N) {
            int kk;

            for (kk = 0 ; kk < N-M ; ++kk) {
                y = (_state[kk] & UPPER_MASK) | (_state[kk+1] & LOWER_MASK);
                _state[kk] = _state[kk+M] ^ (y >>> 1) ^ MAG01[y & 1];
            }

            for ( ; kk < N-1 ; ++kk) {
                y = (_state[kk] & UPPER_MASK) | (_state[kk+1] & LOWER_MASK);
                _state[kk] = _state[kk + (M-N)] ^ (y >>> 1) ^ MAG01[y & 1];
            }

            y = (_state[N-1] & UPPER_MASK) | (_state[0] & LOWER_MASK);
            _state[N-1] = _state[M-1] ^ (y >>> 1) ^ MAG01[y & 1];
            _index = 0;
        }

        y = _state[_index++];

        // Tempering
        y ^= (y >>> 11);
        y ^= (y << 7) & 0x9d2c5680;
        y ^= (y << 15) & 0xefc60000;
        y ^= (y >>> 18);

        return y >>> (32 - bits);
    }
}
