/* edght.f -- translated by f2c (version 19990311).
*/

#include <ivp_physics.hxx>

#include <geompack.hxx>


void IVP_Geompack::edght_(
			  int	*a,
			  int	*b,
			  int	*v,
			  int	*hdfree,
			  int	*last,
			  int	*w
			 ) {

    register int bptr, newp, k, aa, bb, ptr;

    int *edge = this->g_intworkarray;
    int max_n_edges = (int)(this->size_intworkarray / 4)-1; // max_n_edges = size of int work array / 4 because we have to store 4 values for each edge!


/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Search in hash table HASHTABLE for record in EDGE containing */
/*        key (A,B). */

/*     Input parameters: */
/*        A,B - vertex indices, > 0, of edge (also key of hash table) */
/*        V - value associated with edge */
/*        N - upper bound on A, B */
/*        HASHTABLE_SIZE - size of hash table HASHTABLE */
/*        MAX_N_EDGES - maximum size available for EDGE array */
/*        HDFREE - head pointer to linked list of free entries of EDGE */
/*              array due to deletions */
/*        LAST - index of last entry used in EDGE array */
/*        HASHTABLE(0:HASHTABLE_SIZE-1) - hash table of head pointers (direct chaining */
/*              with ordered lists is used) */
/*        EDGE(1:4,1:MAX_N_EDGES) - entries of hash table records; */
/*              EDGE(1,I) = MIN(A,B); EDGE(2,I) = MAX(A,B); */
/*              EDGE(3,I) = V; EDGE(4,I) = link */
/*        [Note: Before first call to this routine, HDFREE, LAST, and */
/*              entries of HASHTABLE should be set to 0.] */

/*     Updated parameters: */
/*        HDFREE,LAST - at least one of these is updated */
/*        HASHTABLE,EDGE - if key with A,B is found then this record is deleted */
/*              from hash table, else record is inserted in hash table */

/*     Output parameters: */
/*        W - EDGE(3,INDEX), where INDEX is index of record, if found; */
/*              else 0 */

/*     Abnormal return: */
/*        IERR is set to 1 */



    /* Function Body */
    if (*a < *b) {
	aa = *a;
	bb = *b;
    } else {
	aa = *b;
	bb = *a;
    }
    k = (aa * this->n_original_vertices + bb) % this->hashtable_size;
    bptr = -1;
    ptr = this->g_hashtable[k];
L10:
    if (ptr != 0) {
	if (edge[(ptr << 2) + 1] > aa) {
	    goto L20;
	} else if (edge[(ptr << 2) + 1] == aa) {
	    if (edge[(ptr << 2) + 2] > bb) {
		goto L20;
	    } else if (edge[(ptr << 2) + 2] == bb) {
		if (bptr == -1) {
		    this->g_hashtable[k] = edge[(ptr << 2) + 4];
		} else {
		    edge[(bptr << 2) + 4] = edge[(ptr << 2) + 4];
		}
		edge[(ptr << 2) + 4] = *hdfree;
		*hdfree = ptr;
		*w = edge[(ptr << 2) + 3];
		return;
	    }
	}
	bptr = ptr;
	ptr = edge[(ptr << 2) + 4];
	goto L10;
    }
L20:
    if (*hdfree > 0) {
	newp = *hdfree;
	*hdfree = edge[(*hdfree << 2) + 4];
    } else {
	(*last)++;
	newp = *last;
recheck_size:
	if (newp >= max_n_edges) {
	    int res = increase_memory((void **)&this->g_intworkarray, &this->size_intworkarray, sizeof(int));
	    if ( res == 0 ) {
		this->ierr = 500;
		return;
	    }
//	    *addr_of_size_intworkarray *= 2;
	    this->size_intworkarray += 1024;
	    edge = this->g_intworkarray;
	    max_n_edges = (int)(this->size_intworkarray / 4)-1;
	    goto recheck_size;
	}
    }
    if (bptr == -1) {
	this->g_hashtable[k] = newp;
    } else {
	edge[(bptr << 2) + 4] = newp;
    }
    edge[(newp << 2) + 1] = aa;
    edge[(newp << 2) + 2] = bb;
    edge[(newp << 2) + 3] = *v;
    edge[(newp << 2) + 4] = ptr;
    *w = 0;

    return;
}
