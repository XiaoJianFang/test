/* sort/gsl_sort_uint.h
 * 
 * Copyright (C) 1996, 1997, 1998, 1999, 2000 Thomas Walter, Brian Gough
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#ifndef __GSL_SORT_UINT_H__
#define __GSL_SORT_UINT_H__

#include <stdlib.h>
#include "gsl_errno.h>
#include "gsl_permutation.h>
#include "gsl_types.h>

#undef __BEGIN_DECLS
#undef __END_DECLS
#ifdef __cplusplus
# define __BEGIN_DECLS extern "C" {
# define __END_DECLS }
#else
# define __BEGIN_DECLS /* empty */
# define __END_DECLS /* empty */
#endif

__BEGIN_DECLS

GSL_EXPORT void gsl_sort_uint (unsigned int * data, const size_t stride, const size_t n);
GSL_EXPORT void gsl_sort_uint_index (size_t * p, const unsigned int * data, const size_t stride, const size_t n);

GSL_EXPORT int gsl_sort_uint_smallest (unsigned int * dest, const size_t k, const unsigned int * src, const size_t stride, const size_t n);
GSL_EXPORT int gsl_sort_uint_smallest_index (size_t * p, const size_t k, const unsigned int * src, const size_t stride, const size_t n);

GSL_EXPORT int gsl_sort_uint_largest (unsigned int * dest, const size_t k, const unsigned int * src, const size_t stride, const size_t n);
GSL_EXPORT int gsl_sort_uint_largest_index (size_t * p, const size_t k, const unsigned int * src, const size_t stride, const size_t n);

__END_DECLS

#endif /* __GSL_SORT_UINT_H__ */
