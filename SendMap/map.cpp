/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Global map (grid-based)
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map.c 1713 2003-08-23 04:03:43Z inspectorg $
**************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include "map.h"
#include "SendMap.h"
extern MapZero MapZeroGps;


// Create a new map
map_t *map_alloc(void)
{
  map_t *map;

  map = (map_t*) malloc(sizeof(map_t));

  // Assume we start at (0, 0)
  map->origin_x = 0;
  map->origin_y = 0;
  
  // Make the size odd
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;
  
  // Allocate storage for main map
  map->cells = (map_cell_t*) NULL;
  
  return map;
}


// Destroy a map
void map_free(map_t *map)
{
  free(map->cells);
  free(map);
  return;
}


// Get the cell at the given point
map_cell_t *map_get_cell(map_t *map, double ox, double oy, double oa)
{
  int i, j;
  map_cell_t *cell;

  i = MAP_GXWX(map, ox);
  j = MAP_GYWY(map, oy);
  
  if (!MAP_VALID(map, i, j))
    return NULL;

  cell = map->cells + MAP_INDEX(map, i, j);
  return cell;
}

template <class Type>  inline Type stringToNum(const std::string& str)
{
	std::istringstream iss(str);
	Type num;
	iss >> num;
	return num;
}
int split2(const std::string& str, std::vector<std::string>& ret_, std::string sep = ",")
{
	if (str.empty())
		return 0;

	ret_.clear();
	std::string tmp;
	std::string::size_type pos_begin = str.find_first_not_of(sep);
	std::string::size_type comma_pos = 0;
	while (pos_begin != std::string::npos)
	{
		comma_pos = str.find(sep, pos_begin);
		if (comma_pos != std::string::npos)
		{
			tmp = str.substr(pos_begin, comma_pos - pos_begin);
			pos_begin = comma_pos + sep.length();
		}
		else
		{
			tmp = str.substr(pos_begin);
			pos_begin = comma_pos;
		}

		if (!tmp.empty())
		{
			ret_.push_back(tmp);
			tmp.clear();
		}
	}
	return 0;
}

int map_load_grid(map_t *map)
{
	map_cell_t *cell;
	int width = 0, height = 0;
	int mapnumber = 0;
	float Scale = 0;
	float xoffset = 0, yoffset = 0;
	long double lat, lon;
	float heading;
	vector<float> x, y;
	vector<int> style;
	std::string FileName;
	cout << "load OCCGridMap..." << endl;
	FileName="/home/xiaojian/test/data/20CM22IbeoVeldGridMap.txt";
	std::string line;
	ifstream  in(FileName);
	if (!in)
	{
		std::cout << "file does not exist." << std::endl;
		return 0;
	}
	while (in.peek() != EOF)
	{

		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split2(line, lineSplit, "\t");
		if (lineSplit.empty()) break;
		if (lineSplit[0] == "[MAPNUM]")
		{
			mapnumber = stringToNum<int>(lineSplit[1]);
		}
		else if (lineSplit[0] == "[SCALE]")
		{
			Scale = stringToNum<float>(lineSplit[1]);
			MapZeroGps.scale = Scale;
		}
		else if (lineSplit[0] == "[GPS]")
		{
			lat = stringToNum<long double>(lineSplit[1]);
			lon = stringToNum<long double>(lineSplit[2]);
			heading = stringToNum<float>(lineSplit[3]);

			lat = lat / (long double)180 * (long double)PI64;
			lon = lon / (long double)180 * (long double)PI64;
			heading = heading / (long double)180 * (long double)PI64;
			MapZeroGps.lat = lat;
			MapZeroGps.lon = lon;
			MapZeroGps.head = heading;
		}
		else if (lineSplit[0] == "[XOFFSET]")
		{
			xoffset = stringToNum<float>(lineSplit[1]);
			MapZeroGps.xoffset = -xoffset*Scale;
		}
		else if (lineSplit[0] == "[YOFFSET]")
		{
			yoffset = stringToNum<float>(lineSplit[1]);
			MapZeroGps.yoffset = -yoffset*Scale;

		}
		else if (lineSplit[0] == "[WIDTH]")
		{
			width = stringToNum<int>(lineSplit[1]);
		}
		else if (lineSplit[0] == "[HEIGHT]")
		{
			height = stringToNum<int>(lineSplit[1]);
		}
		else if (lineSplit[0] == "[X]")
		{

			for (size_t i = 1; i < lineSplit.size(); i++)
			{
				x.push_back(stringToNum<float>(lineSplit[i]));
			}
			std::getline(in, line);
			split2(line, lineSplit, "\t");
			for (size_t i = 1; i < lineSplit.size(); i++)
			{
				y.push_back(stringToNum<float>(lineSplit[i]));
			}

			std::getline(in, line);
			split2(line, lineSplit, "\t");
			for (size_t i = 1; i < lineSplit.size(); i++)
			{
				style.push_back(stringToNum<int>(lineSplit[i]));
			}

		}
	}

	// Allocate space in the map
	if (map->cells == NULL)
	{
		map->origin_x = MapZeroGps.xoffset;
		map->origin_y = MapZeroGps.yoffset;
		map->scale = Scale;
		map->size_x = width;
		map->size_y = height;
		map->cells = (map_cell_t*)calloc(width * height, sizeof(map->cells[0]));
	}
	else
	{
		if (width != map->size_x || height != map->size_y)
		{
			//PLAYER_ERROR("map dimensions are inconsistent with prior map dimensions");
			return -1;
		}
	}

	for (size_t i = 0; i < x.size(); i++)
	{
		int i_, j_;
		i_ = x[i];
		j_ = y[i];
		if (!MAP_VALID(map, i_, j_))
			continue;
		cell = map->cells + MAP_INDEX(map, i_, j_);
		cell->occ_state = style[i];
	}

	return 0;
}

int map_load_gridsub(map_t *map,std::string dir)
{
    //map_cell_t *cell;
    int width = 0, height = 0;
    int mapnumber = 0;
    float Scale = 0;
    float submapscale = 10;
    float xoffset = 0, yoffset = 0;
    long double lat, lon;
    float heading;
    vector<int> x, y;
    vector<int> indexx, indexy;
    vector<int> style;
    std::string FileName;
    FileName="./data/"+dir;
    cout << "load "<<FileName << endl;

    std::string line;
    ifstream  in(FileName);
    if (!in)
    {
        std::cout << FileName<<" does not exist!!!" << std::endl;
        return -1;
    }
    while (in.peek() != EOF)
    {

        std::vector<std::string> lineSplit;
        std::getline(in, line);
        split2(line, lineSplit, "\t");
        if (lineSplit.empty()) break;
        if (lineSplit[0] == "[MAPNUM]")
        {
            mapnumber = stringToNum<int>(lineSplit[1]);
        }
        else if (lineSplit[0] == "[SCALE]")
        {
            Scale = stringToNum<float>(lineSplit[1]);
            MapZeroGps.scale = Scale;
        }
        else if (lineSplit[0] == "[INDEXSCALE]")
        {
            submapscale = stringToNum<int>(lineSplit[1]);
            //MapZeroGps.scale = submapscale;
        }
        else if (lineSplit[1] == "[GPS]")
        {
            lat = stringToNum<long double>(lineSplit[3]);
            lon = stringToNum<long double>(lineSplit[4]);
            heading = stringToNum<long double>(lineSplit[5]);

            lat = lat / (long double)180 * (long double)PI64;
            lon = lon / (long double)180 * (long double)PI64;
            heading = heading / (long double)180 * (long double)PI64;
            MapZeroGps.lat = lat;
            MapZeroGps.lon = lon;
            MapZeroGps.head = heading;
        }
        else if (lineSplit[0] == "[XOFFSET]")
        {
            xoffset = stringToNum<float>(lineSplit[1]);
            MapZeroGps.xoffset = -xoffset*Scale;
        }
        else if (lineSplit[0] == "[YOFFSET]")
        {
            yoffset = stringToNum<float>(lineSplit[1]);
            MapZeroGps.yoffset = -yoffset*Scale;

        }
        else if (lineSplit[0] == "[WIDTH]")
        {
            width = stringToNum<int>(lineSplit[1]);
        }
        else if (lineSplit[0] == "[HEIGHT]")
        {
            height = stringToNum<int>(lineSplit[1]);
        }
        else if (lineSplit[0] == "[X]")
        {

            for (size_t i = 1; i < lineSplit.size(); i++)
            {
                x.push_back(stringToNum<int>(lineSplit[i]));
            }
            std::getline(in, line);
            split2(line, lineSplit, "\t");
            for (size_t i = 1; i < lineSplit.size(); i++)
            {
                y.push_back(stringToNum<int>(lineSplit[i]));
            }

            std::getline(in, line);
            split2(line, lineSplit, "\t");
            for (size_t i = 1; i < lineSplit.size(); i++)
            {
                style.push_back(stringToNum<int>(lineSplit[i]));
            }

        }
        else if (lineSplit[0] == "[INDEX]")
        {

            for (size_t i = 1; i < lineSplit.size(); i++)
            {
                indexx.push_back(stringToNum<int>(lineSplit[i]));
            }
            std::getline(in, line);
            split2(line, lineSplit, "\t");
            for (size_t i = 1; i < lineSplit.size(); i++)
            {
                indexy.push_back(stringToNum<int>(lineSplit[i]));
            }
        }
    }
    // Allocate space in the map
    if (map->cells == NULL)
    {
        map->origin_x = MapZeroGps.xoffset;
        map->origin_y = MapZeroGps.yoffset;
        map->scale = Scale;
        map->size_x = width;
        map->size_y = height;
        map->submapscale = (int)(submapscale / Scale);
        map->submapsize = map->submapscale*map->submapscale;
        map->submapindexsize_x = (int)(width / map->submapscale+1);
        map->submapindexsize_y = (int)(height / map->submapscale+1);

        map->cells = (map_cell_t*)calloc((indexy.size() + map->submapindexsize_x + map->submapindexsize_y)*map->submapsize, sizeof(map->cells[0]));
        map->submapindex = (int*)calloc((map->submapindexsize_x *map->submapindexsize_y), sizeof(int));
    }
    else
    {
        if (width != map->size_x || height != map->size_y)
        {
            //PLAYER_ERROR("map dimensions are inconsistent with prior map dimensions");
            return -1;
        }
    }
    vector<int> submaplistx, submaplisty,submaplistindex;

    int submapindex_ = 0;
    int submapindexcount = -1;
    int mapindex = 0;
    //for (size_t i = 0; i < indexx.size(); i++)
    //{
    //	int index_i, index_j;
    //
    //	int *saveindex;
    //	index_i = int((int)(indexx[i] / Scale)/ map->submapscale);
    //	index_j = int((int)(indexy[i] / Scale)/ map->submapscale);
    //	if (!SUBMAPINDEX_VALID(map, index_i, index_j))
    //		continue;
    //	//submapindex_ = SUBMAPINDEX_INDEX(map, index_i, index_j);
    ////	saveindex = map->submapindex + submapindex_;
    //	saveindex = map->submapindex + SUBMAPINDEX_INDEX(map, index_i, index_j);
    //	*saveindex = mapindex;
    //	mapindex++;
    //}

    for (size_t i = 0; i < x.size(); i++)
    {
        int subi_, subj_;
        int index_i, index_j;
        int *saveindex;
        map_cell_t *cell;
        index_i = int(x[i] / map->submapscale);
        index_j = int(y[i] / map->submapscale);

        subi_ = x[i] % map->submapscale;
        subj_ = y[i] % map->submapscale;
        if (!SUBMAPINDEX_VALID(map, index_i, index_j))
            continue;
        if (submaplistx.size() < 1)
        {
            submapindexcount++;
            submaplistx.push_back(index_i);
            submaplisty.push_back(index_j);
            submaplistindex.push_back(submapindexcount);
            submapindex_ = submapindexcount;
        }
        else
        {
            int flag = 0;
            for (int ii = submaplistx.size()-1; ii >=0; ii--)
            {
                if (submaplistx[ii]==index_i&&submaplisty[ii] == index_j)
                {
                    flag++;
                    submapindex_ = submaplistindex[ii];
                    break;
                }
            }

            if (!flag)
            {
                submapindexcount++;
                submaplistx.push_back(index_i);
                submaplisty.push_back(index_j);
                submaplistindex.push_back(submapindexcount);
                submapindex_ = submapindexcount;
            }
        }

        saveindex = map->submapindex + SUBMAPINDEX_INDEX(map, index_i, index_j);
        *saveindex = submapindex_;
        //saveindex = map->submapindex + SUBMAPINDEX_INDEX(map, index_i, index_j);

        if (!SUBMAP_VALID(map, subi_, subj_))
            continue;

        cell = map->cells + *saveindex*map->submapsize + SUBMAP_INDEX(map, subi_, subj_);
        cell->occ_state = style[i];
    }
    cout << "submapnumber="<<submapindexcount << endl;
    return 1;
}

