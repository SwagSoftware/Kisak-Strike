/**
 * Shortest Distance Utilities. Some static helper functions to help calculate
 * shortest distances
 */
class ShortestDistanceUtil
{
public:
	/**
	 * Calculate the shortest distance from a point to an edge (line segment)
	 * @param p point
     * @param s1 edge start point
	 * @param s2 edge end point
	 * @param C point on edge closest to p
	 */
	static inline void PointToEdge(const IVP_U_Point* p, const IVP_U_Point* s1, const IVP_U_Point* s2, IVP_U_Point* C)
	{
		IVP_U_Point l(s1), d;
		l.subtract(p);
		d.subtract(s2, s1);

		IVP_DOUBLE t = -(l.dot_product(&d) / d.dot_product(&d));

		if(t < 0.0f || t > 1.0f)
		{
			if(t < 0.0f)
			{
				t = 0.0f;
			}
			else
			{
				t = 1.0f;
			}
		}

		C->add_multiple(s1, &d, t);
		return;
	};

	/**
	 * Calculate the shortest distance from a point to a triangle
	 * @param p point
     * @param t1 triangle vertex 1
	 * @param t2 triangle vertex 2
	 * @param t3 triangle vertex 3
	 * @param C point on triangle closest to p
	 */
	static inline void PointToTriangle(const IVP_U_Point* p, const IVP_U_Point* t1, const IVP_U_Point* t2, const IVP_U_Point* t3, IVP_U_Point* C)
	{
    	// calculate edges
    	IVP_U_Point edge1, edge2, edge3;
    	edge1.add_multiple(t2, t1, -1.0f);
    	edge2.add_multiple(t3, t1, -1.0f);
    	edge3.add_multiple(t3, t2, -1.0f);
    
    	for(int i = 0; i < 1; i++)
    	{ // ray triangle intersection
    		// ray direction
    		IVP_U_Point dir; dir.inline_calc_cross_product_and_normize(&edge2, &edge1);
    
    		IVP_U_Point h; h.inline_calc_cross_product(&dir, &edge2); 
    		IVP_DOUBLE det = edge1.dot_product(&h);
    
    		if(det > -0.00001f && det < 0.00001f)
    		{
    			return;
    		}
    
    		IVP_DOUBLE inv_det = 1.0f/det;
    		IVP_U_Point s; s.add_multiple(p, t1, -1.0f);
    		IVP_DOUBLE u = inv_det * s.dot_product(&h);
    
    		IVP_U_Point q; q.inline_calc_cross_product(&s, &edge1);
    		IVP_DOUBLE v = inv_det * dir.dot_product(&q);
    
    		// set C to be the intersection point of the ray and the triangle
    		C->set_multiple(t1, 1.0f-u-v);
    		C->add_multiple(t2, u);
    		C->add_multiple(t3, v);
    
    		// not inside triangle
    		if(u < 0.0f || u > 1.0f)
    			break;
    
    		// not inside triangle
    		if(v < 0.0f || v > 1.0f)
    			break;
    
    		// not inside triangle
    		if(u + v > 1.0f)
    			break;
    
    		return;
    	}
    
    	{ // contact point is triangle edge, deal with that accordingly
    		IVP_U_Point cp1, cp2, cp3;

			ShortestDistanceUtil::PointToEdge(C, t1, t2, &cp1);
    		ShortestDistanceUtil::PointToEdge(C, t1, t3, &cp2);
    		ShortestDistanceUtil::PointToEdge(C, t2, t3, &cp3);
    
    		IVP_U_Point dv1; dv1.add_multiple(C, &cp1, -1.0f);
    		IVP_U_Point dv2; dv2.add_multiple(C, &cp2, -1.0f);
    		IVP_U_Point dv3; dv3.add_multiple(C, &cp3, -1.0f);
    
    		IVP_DOUBLE ds1 = dv1.quad_length();
    		IVP_DOUBLE ds2 = dv2.quad_length();
    		IVP_DOUBLE ds3 = dv3.quad_length();
    
    		if(ds1 <= ds2 && ds1 <= ds3)
    			C->set(&cp1);
    		else if(ds2 <= ds1 && ds2 <= ds3)
    			C->set(&cp2);
    		else if(ds3 <= ds1 && ds3 <= ds1)
    			C->set(&cp3);
    		else
    			CORE;
    	}
    
    	return;
    };
};

