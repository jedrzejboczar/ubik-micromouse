#pragma once

/*
 * Code taken from:
 * https://rosettacode.org/wiki/Maze_generation
 */

#include <iostream>
#include <string>
#include <cstdint>

namespace maze_gen {

using namespace std;

enum directions { NONE, NOR = 1, EAS = 2, SOU = 4, WES = 8 };


class MazeGenerator
{
public:
    MazeGenerator() { _world = 0; }

    ~MazeGenerator() { killArray(); }

    void create( int size )
    {
        _s = size;
        generate();
    }

    void generate()
    {
        killArray();
        _world = new uint8_t[_s * _s];
        memset(_world, 0, _s * _s);
        _ptX = rand() % _s; _ptY = rand() % _s;
        carve();
    }

    void carve()
    {
        while( true )
        {
            int d = getDirection();
            if( d < NOR ) return;

            switch( d )
            {
                case NOR:
                    _world[_ptX + _s * _ptY] |= NOR; _ptY--;
                    _world[_ptX + _s * _ptY] = SOU | SOU << 4;
                    break;
                case EAS:
                    _world[_ptX + _s * _ptY] |= EAS; _ptX++;
                    _world[_ptX + _s * _ptY] = WES | WES << 4;
                    break;
                case SOU:
                    _world[_ptX + _s * _ptY] |= SOU; _ptY++;
                    _world[_ptX + _s * _ptY] = NOR | NOR << 4;
                    break;
                case WES:
                    _world[_ptX + _s * _ptY] |= WES; _ptX--;
                    _world[_ptX + _s * _ptY] = EAS | EAS << 4;
            }
        }
    }

    int getDirection()
    {
        int d = 1 << rand() % 4;
        while( true )
        {
            for( int x = 0; x < 4; x++ )
            {
                if( testDir( d ) ) return d;
                d <<= 1;
                if( d > 8 ) d = 1;
            }
            d = ( _world[_ptX + _s * _ptY] & 0xf0 ) >> 4;
            if( !d ) return -1;
            switch( d )
            {
                case NOR: _ptY--; break;
                case EAS: _ptX++; break;
                case SOU: _ptY++; break;
                case WES: _ptX--; break;
            }
            d = 1 << rand() % 4;
        }
    }

    bool testDir( int d )
    {
        switch( d )
        {
            case NOR: return ( _ptY - 1 > -1 && !_world[_ptX + _s * ( _ptY - 1 )] );
            case EAS: return ( _ptX + 1 < _s && !_world[_ptX + 1 + _s * _ptY] );
            case SOU: return ( _ptY + 1 < _s && !_world[_ptX + _s * ( _ptY + 1 )] );
            case WES: return ( _ptX - 1 > -1 && !_world[_ptX - 1 + _s * _ptY] );
        }
        return false;
    }

    void killArray() { if( _world ) delete [] _world; }

    uint8_t*    _world;
    int      _s, _ptX, _ptY;
};

} // namespace maze_gen
