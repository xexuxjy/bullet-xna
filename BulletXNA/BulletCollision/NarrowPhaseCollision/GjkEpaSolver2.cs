/*
 * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

using System;
using System.Collections.Generic;

using Microsoft.Xna.Framework;
using BulletXNA.BulletCollision.CollisionShapes;

namespace BulletXNA.BulletCollision.NarrowPhaseCollision
{
    public class GjkEpaSolver2
    {
        public static bool debugGJK = false;

        public static void Initialize(ConvexShape shape0,ref Matrix wtrs0,
            ConvexShape shape1,ref Matrix wtrs1,
            GjkEpaSolver2Results results,
            GjkEpaSolver2MinkowskiDiff shapeR,
            bool withmargins)
        {
            /* Results		*/ 
            results.witnesses0 = Vector3.Zero;
            results.witnesses1 = Vector3.Zero;
            results.status = GjkEpaSolver2Status.Separated;
            /* Shape		*/ 
            shapeR.m_shapes[0] =	shape0;
            shapeR.m_shapes[1] =	shape1;

            shapeR.m_toshape1 = MathUtil.TransposeTimesBasis(ref wtrs1, ref wtrs0);
            shapeR.m_toshape0 = MathUtil.InverseTimes(ref wtrs0, ref wtrs1);




            if (BulletGlobals.g_streamWriter != null && debugGJK)
            {
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "gjksolver2::init::shape0", shapeR.m_toshape0);
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "gjksolver2::init::WTRS0", wtrs0);
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "gjksolver2::init::WTRS1", wtrs1);

                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "gjksolver2::init::shape1", shapeR.m_toshape1);
            }


            shapeR.EnableMargin(withmargins);
        }

        
        
        public static bool	Distance(ConvexShape shape0,ref Matrix wtrs0,ConvexShape shape1,ref Matrix wtrs1,ref Vector3 guess,GjkEpaSolver2Results	results)
        {
            GjkEpaSolver2MinkowskiDiff shape = new GjkEpaSolver2MinkowskiDiff();
            Initialize(shape0,ref wtrs0,shape1,ref wtrs1,results,shape,false);
            GJK	gjk = new GJK();
            GJKStatus gjk_status= gjk.Evaluate(shape,ref guess);
            if(gjk_status == GJKStatus.Valid)
            {
                Vector3	w0 = Vector3.Zero;
                Vector3	w1 = Vector3.Zero;
                for(uint i=0;i<gjk.m_simplex.rank;++i)
                {
                    float p=gjk.m_simplex.p[i];
                    w0+=shape.Support(ref gjk.m_simplex.c[i].d,0)*p;
                    Vector3 temp = -gjk.m_simplex.c[i].d;
                    w1+=shape.Support(ref temp,1)*p;
                }
                results.witnesses0	= Vector3.Transform(w0,wtrs0);
                results.witnesses1	= Vector3.Transform(w1,wtrs0);
                results.normal = w0-w1;
                results.distance =	results.normal.Length();
                results.normal	/=	results.distance>GJK_MIN_DISTANCE?results.distance:1;
                return(true);
            }
            else
            {
                //GjkEpaSolver2Status
                results.status = (gjk_status==GJKStatus.Inside)?GjkEpaSolver2Status.Penetrating :GjkEpaSolver2Status.GJK_Failed	;
                return(false);
            }
        }

        public static bool Penetration(ConvexShape shape0, ref Matrix wtrs0, ConvexShape shape1, ref Matrix wtrs1, ref Vector3 guess, GjkEpaSolver2Results results)
        {
            return Penetration(shape0, ref wtrs0, shape1, ref wtrs1, ref guess, results, true);
        }

        public static bool Penetration(ConvexShape shape0,ref Matrix wtrs0,ConvexShape shape1,ref Matrix wtrs1,ref Vector3 guess,GjkEpaSolver2Results results,bool usemargins)
        {
            GjkEpaSolver2MinkowskiDiff shape = new GjkEpaSolver2MinkowskiDiff();
            Initialize(shape0,ref wtrs0,shape1,ref wtrs1,results, shape,usemargins);
            GJK	gjk = new GJK();	
            Vector3 minusGuess = -guess;
            GJKStatus	gjk_status=gjk.Evaluate(shape,ref minusGuess);
            switch(gjk_status)
            {
            case GJKStatus.Inside:
                {
                    EPA	epa = new EPA();
                    eStatus	epa_status=epa.Evaluate(gjk,ref minusGuess);
                    if(epa_status!=eStatus.Failed)
                    {
                        Vector3	w0 = Vector3.Zero;
                        for(uint i=0;i<epa.m_result.rank;++i)
                        {
                            // order of results here is 'different' , EPA.evaluate.
                            w0+=shape.Support(ref epa.m_result.c[i].d,0)*epa.m_result.p[i];
                        }
                        results.status			=	GjkEpaSolver2Status.Penetrating;
                        results.witnesses0	=	Vector3.Transform(w0,wtrs0);
                        results.witnesses1	=	Vector3.Transform((w0-epa.m_normal*epa.m_depth),wtrs0);
                        results.normal			=	-epa.m_normal;
                        results.distance		=	-epa.m_depth;
                        return(true);
                    } else results.status=GjkEpaSolver2Status.EPA_Failed;
                }
                break;
            case GJKStatus.Failed:
                results.status=GjkEpaSolver2Status.GJK_Failed;
                break;
            }
            return(false);
        }

        //
        public float SignedDistance(ref Vector3 position, float margin, ConvexShape shape0, ref Matrix wtrs0, GjkEpaSolver2Results results)
        {
            GjkEpaSolver2MinkowskiDiff shape = new GjkEpaSolver2MinkowskiDiff();
            SphereShape	shape1 = new SphereShape(margin);
            Matrix wtrs1 = Matrix.CreateFromQuaternion(Quaternion.Identity);
            wtrs0.Translation = position;
	        
            Initialize(shape0,ref wtrs0,shape1,ref wtrs1,results,shape,false);
            GJK	gjk = new GJK();	
            Vector3 guess = new Vector3(1,1,1);
            GJKStatus	gjk_status=gjk.Evaluate(shape,ref guess);
            if(gjk_status==GJKStatus.Valid)
            {
                Vector3	w0=Vector3.Zero;
                Vector3	w1=Vector3.Zero;
                for(int i=0;i<gjk.m_simplex.rank;++i)
                {
                    float p=gjk.m_simplex.p[i];
                    w0+=shape.Support( ref gjk.m_simplex.c[i].d,0)*p;
                    Vector3 temp = -gjk.m_simplex.c[i].d;
                    w1+=shape.Support(ref temp,1)*p;
                }
                results.witnesses0 = Vector3.Transform(w0,wtrs0);
                results.witnesses1 = Vector3.Transform(w1,wtrs0);
                Vector3	delta=	results.witnesses1-results.witnesses0;
                float margin2 = shape0.GetMarginNonVirtual()+shape1.GetMarginNonVirtual();
                float length = delta.Length();	
                results.normal = delta/length;
                results.witnesses0 +=	results.normal*margin2;
                return(length-margin2);
            }
            else
            {
                if(gjk_status==GJKStatus.Inside)
                {
                    if(Penetration(shape0,ref wtrs0,shape1,ref wtrs1,ref gjk.m_ray,results))
                    {
                        Vector3	delta=	results.witnesses0-results.witnesses1;
                        float length= delta.Length();
                        if (length >= MathUtil.SIMD_EPSILON)
                            results.normal	=	delta/length;			
                        return(-length);
                    }
                }	
            }
            return(MathUtil.SIMD_INFINITY);
        }

        //
        public bool SignedDistance(ConvexShape	shape0,ref Matrix wtrs0,ConvexShape shape1,ref Matrix wtrs1,ref Vector3 guess,GjkEpaSolver2Results results)
        {
            if(!Distance(shape0,ref wtrs0,shape1,ref wtrs1,ref guess,results))
                return(Penetration(shape0,ref wtrs0,shape1,ref wtrs1,ref guess,results,false));
            else
                return(true);
        }


        /* GJK	*/
        public const int GJK_MAX_ITERATIONS = 128;
        public const float GJK_ACCURARY = 0.0001f;
        public const float GJK_MIN_DISTANCE = 0.0001f;
        public const float GJK_DUPLICATED_EPS = 0.0001f;
        public const float GJK_SIMPLEX2_EPS = 0f;
        public const float GJK_SIMPLEX3_EPS = 0f;
        public const float GJK_SIMPLEX4_EPS = 0f;

        /* EPA	*/
        public const int EPA_MAX_VERTICES = 64;
        public const int EPA_MAX_FACES = EPA_MAX_VERTICES * 2;
        public const int EPA_MAX_ITERATIONS = 255;
        public const float EPA_ACCURACY = 0.0001f;
        public const float EPA_FALLBACK = 10 * EPA_ACCURACY;
        public const float EPA_PLANE_EPS = 0.00001f;
        public const float EPA_INSIDE_EPS = 0.01f;

    }

    public enum GjkEpaSolver2Status
    {
        Separated,		/* Shapes doesnt penetrate												*/ 
        Penetrating,	/* Shapes are penetrating												*/ 
        GJK_Failed,		/* GJK phase fail, no big issue, shapes are probably just 'touching'	*/ 
        EPA_Failed		/* EPA phase fail, bigger problem, need to save parameters, and debug	*/ 
    }		

    public class GjkEpaSolver2Results
    {
        public GjkEpaSolver2Status status;
        public Vector3 witnesses0;
        public Vector3 witnesses1;
        public Vector3	normal;
        public float distance;
    }


        // MinkowskiDiff
    public class GjkEpaSolver2MinkowskiDiff
    {
        public GjkEpaSolver2MinkowskiDiff()
        {
        }

        public void EnableMargin(bool enable)
        {
            m_enableMargin = enable;
        }	
        public Vector3 Support0(ref Vector3 d)
        {
            if(m_enableMargin)
            {
                return m_shapes[0].LocalGetSupportVertexNonVirtual(ref d);
            }
            return m_shapes[0].LocalGetSupportVertexWithoutMarginNonVirtual(ref d);
        }

        public Vector3 Support1(ref Vector3 d)
        {
            Vector3 dcopy = Vector3.TransformNormal(d, m_toshape1);
            Vector3 temp = m_enableMargin?m_shapes[1].LocalGetSupportVertexNonVirtual(ref dcopy) :
                                            m_shapes[1].LocalGetSupportVertexWithoutMarginNonVirtual(ref dcopy);

            return Vector3.Transform(temp,m_toshape0);
        }

        public Vector3 Support(ref Vector3 d)
        {
            Vector3 minusD = -d;
            Vector3 temp = Support1(ref minusD);
            return(Support0(ref d)-temp);
        }
        
        public Vector3	Support(ref Vector3 d,uint index)
        {
            if(index > 0)
                return(Support1(ref d));
            else
                return(Support0(ref d));
        }

        public bool m_enableMargin;
        public ConvexShape[] m_shapes = new ConvexShape[2];
        public Matrix m_toshape1 = Matrix.Identity;
        public Matrix m_toshape0 = Matrix.Identity;

    }

    public struct sSV
    {
        public Vector3 d;
        public Vector3 w;
    }

    public class sSimplex
    {
        public sSimplex()
        {
            //for (int i = 0; i < c.Length; ++i)
            //{
            //    c[i] = new sSV();
            //}
        }
        public sSV[] c = new sSV[4];
        public float[] p = new float[4];
        public uint rank;
    }

    public enum GJKStatus	
    { 
        Valid,
        Inside,
        Failed		
    }

    public class GJK
    {
        public GJK()
        {
            Initialise();
        }
        
        public void Initialise()
        {
            m_ray = Vector3.Zero;
            m_nfree		=	0;
            m_status	=	GJKStatus.Failed;
            m_current	=	0;
            m_distance	=	0f;
            for (int i = 0; i < m_simplices.Length; ++i)
            {
                m_simplices[i] = new sSimplex();
            }

            for (int i = 0; i < m_store.Length; ++i)
            {
                m_store[i] = new sSV();
            }
            // Free is a list of pointers/references so doesn't need objects.
            //for (int i = 0; i < m_free.Length; ++i)
            //{
            //    m_free[i] = new sSV();
            //}
        }

        public GJKStatus Evaluate(GjkEpaSolver2MinkowskiDiff shapearg, ref Vector3 guess)
        {
            uint iterations=0;
            float sqdist=0f;
            float alpha=0f;
            Vector3[] lastw = new Vector3[4];
            uint clastw=0;
            /* Initialize solver		*/ 
            m_free[0] =	m_store[0];
            m_free[1] =	m_store[1];
            m_free[2] =	m_store[2];
            m_free[3] =	m_store[3];
            m_nfree	= 4;
            m_current =	0;
            m_status = GJKStatus.Valid;
            m_shape	= shapearg;
            m_distance = 0f;
            /* Initialize simplex		*/ 
            m_simplices[0].rank	= 0;
            m_ray =	guess;
            float sqrl=	m_ray.LengthSquared();
            Vector3 temp = sqrl>0?-m_ray:new Vector3(1,0,0);
            AppendVertice(m_simplices[0],ref temp);
            m_simplices[0].p[0]	= 1;
            m_ray =	m_simplices[0].c[0].w;	
            sqdist =	sqrl;
            lastw[0]=lastw[1]=lastw[2]=lastw[3]	=	m_ray;
            /* Loop						*/ 
            do	
            {
                uint next = 1-m_current;
                sSimplex cs = m_simplices[m_current];
                sSimplex ns = m_simplices[next];
                /* Check zero							*/ 
                float rl=m_ray.Length();
                if (rl < GjkEpaSolver2.GJK_MIN_DISTANCE)
                {/* Touching or inside				*/ 
                    m_status=GJKStatus.Inside;
                    break;
                }
                /* Append new vertice in -'v' direction	*/ 
                Vector3 temp2 = -m_ray;
                AppendVertice(cs,ref temp2);
                Vector3	w = cs.c[cs.rank-1].w;
                bool found = false;
                for(int i=0;i<4;++i)
                {
                    if ((w - lastw[i]).LengthSquared() < GjkEpaSolver2.GJK_DUPLICATED_EPS)
                    { 
                        found=true;
                        break; 
                    }
                }
                if(found)
                {/* Return old simplex				*/ 
                    RemoveVertice(m_simplices[m_current]);
                    break;
                }
                else
                {/* Update lastw					*/ 
                    lastw[clastw=(clastw+1)&3]=w;
                }
                /* Check for termination				*/ 
                float omega=Vector3.Dot(m_ray,w)/rl;
                alpha=Math.Max(omega,alpha);
                if (((rl - alpha) - (GjkEpaSolver2.GJK_ACCURARY * rl)) <= 0)
                {/* Return old simplex				*/ 
                    RemoveVertice(m_simplices[m_current]);
                    break;
                }		

                /* Reduce simplex						*/ 
                Vector4 weights = new Vector4();
                uint mask=0;
                switch(cs.rank)
                {
                    case 2 :
                    {
                        sqdist=GJK.ProjectOrigin(ref cs.c[0].w,ref cs.c[1].w,ref weights,ref mask);
                        break;
                    }
                    case 3:
                    {
                        sqdist = GJK.ProjectOrigin(ref cs.c[0].w, ref cs.c[1].w, ref cs.c[2].w, ref weights, ref mask);
                        break;
                    }
                    case 4:
                    {
                        sqdist = GJK.ProjectOrigin(ref cs.c[0].w, ref cs.c[1].w, ref cs.c[2].w, ref cs.c[3].w, ref weights, ref mask);
                        break;
                    }
                }
                if(sqdist>=0)
                {/* Valid	*/ 
                    ns.rank	= 0;
                    m_ray =	Vector3.Zero;
                    m_current =	next;
                    for(uint i=0,ni=cs.rank;i<ni;++i)
                    {
                        if((mask&(1<<(int)i)) != 0)
                        {
                            ns.c[ns.rank] =	cs.c[i];
                            float weight = MathUtil.VectorComponent(ref weights, (int)i);
                            ns.p[ns.rank++]	= weight;
                            m_ray += cs.c[i].w * weight;
                        }
                        else
                        {
                            m_free[m_nfree++] =	cs.c[i];
                        }
                    }
                    if(mask==15)
                    {
                        m_status=GJKStatus.Inside;
                    }
                }
                else
                {/* Return old simplex				*/ 
                    RemoveVertice(m_simplices[m_current]);
                    break;
                }
                m_status = ((++iterations) < GjkEpaSolver2.GJK_MAX_ITERATIONS) ? m_status : GJKStatus.Failed;
            } while(m_status==GJKStatus.Valid);

            m_simplex = m_simplices[m_current];
			
            switch(m_status)
            {
                case	GJKStatus.Valid:
                {
                    m_distance=m_ray.Length();
                    break;
                }
                case	GJKStatus.Inside:	
                {
                    m_distance=0;
                    break;
                }
            }


            if (BulletGlobals.g_streamWriter != null && GjkEpaSolver2.debugGJK)
            {
                BulletGlobals.g_streamWriter.WriteLine(String.Format("gjk eval dist[{0}]", m_distance));
            }


            return(m_status);
        }


        public bool	EncloseOrigin()
        {
            switch(m_simplex.rank)
            {
                case	1:
                {
                    for(int i=0;i<3;++i)
                    {
                        Vector3 axis= Vector3.Zero;
                        MathUtil.VectorComponent(ref axis,i,1f);
                        AppendVertice(m_simplex, ref axis);
                        if(EncloseOrigin())
                        {
                            return(true);
                        }
                        RemoveVertice(m_simplex);
                        Vector3 temp = -axis;
                        AppendVertice(m_simplex,ref temp);
                        if(EncloseOrigin())	
                        {
                            return(true);
                        }
                        RemoveVertice(m_simplex);
                    }
                    break;
                }
                case	2:
                {
                    Vector3	d=m_simplex.c[1].w-m_simplex.c[0].w;
                    for(int i=0;i<3;++i)
                    {
                        Vector3 axis= Vector3.Zero;
                        MathUtil.VectorComponent(ref axis,i,1f);
                        Vector3	p= Vector3.Cross(d,axis);
                        if(p.LengthSquared()>0)
                        {
                            AppendVertice(m_simplex, ref p);
                            if(EncloseOrigin())
                            {
                                return(true);
                            }
                            RemoveVertice(m_simplex);
                            Vector3 temp = -p;
                            AppendVertice(m_simplex,ref temp);
                            if(EncloseOrigin())
                            {
                                return(true);
                            }
                            RemoveVertice(m_simplex);
                        }
                    }
                    break;
                }
                case 3:
                {
                    Vector3	n = Vector3.Cross(m_simplex.c[1].w-m_simplex.c[0].w,
                        m_simplex.c[2].w-m_simplex.c[0].w);
                    if(n.LengthSquared()>0)
                    {
                        AppendVertice(m_simplex,ref n);
                        if(EncloseOrigin())	
                        {
                            return(true);
                        }
                        RemoveVertice(m_simplex);
                        Vector3 temp = -n;
                        AppendVertice(m_simplex,ref temp);
                        if(EncloseOrigin())
                        {
                            return(true);
                        }
                        RemoveVertice(m_simplex);
                    }
                    break;
                }
                case 4:
                {
                    if (Math.Abs(GJK.Det(m_simplex.c[0].w - m_simplex.c[3].w,
                        m_simplex.c[1].w-m_simplex.c[3].w,
                        m_simplex.c[2].w-m_simplex.c[3].w))>0)
                        return(true);
                    break;
                }
                default:
                {
                    break;
                }
            }
            return(false);
        }

        public void	GetSupport(ref Vector3 d,ref sSV sv)
        {
            sv.d = d/d.Length();
            sv.w = m_shape.Support(ref sv.d);
        }
        public void RemoveVertice(sSimplex simplex)
        {
            m_free[m_nfree++]=simplex.c[--simplex.rank];
        }

        public void AppendVertice(sSimplex simplex,ref Vector3 v)
        {
            simplex.p[simplex.rank]=0;
            simplex.c[simplex.rank]=m_free[--m_nfree];
            GetSupport(ref v,ref simplex.c[simplex.rank++]);
        }


        public static float Det(Vector3 a, Vector3 b, Vector3 c)
        {
            return Det(ref a, ref b, ref c);
        }

        public static float Det(ref Vector3 a,ref Vector3 b,ref Vector3 c)
        {
            return(	a.Y*b.Z*c.X+a.Z*b.X*c.Y-
                a.X*b.Z*c.Y-a.Y*b.X*c.Z+
                a.X*b.Y*c.Z-a.Z*b.Y*c.X);
        }

        public static float ProjectOrigin(ref Vector3 a,ref Vector3 b,ref Vector4 w,ref uint m)
        {
            Vector3	d=b-a;
            float l=d.LengthSquared();
            if (l > GjkEpaSolver2.GJK_SIMPLEX2_EPS)
            {
                float t = (l>0f?(-Vector3.Dot(a,d)/l):0f);
                if(t>=1)		
                { 
                    w.X=0f;
                    w.Y=1f;
                    m=2;
                    return b.LengthSquared(); 
                }
                else if(t<=0)	
                { 
                    w.X=1f;
                    w.Y=0f;
                    m=1;
                    return a.LengthSquared(); 
                }
                else
                { 
                    w.X=1-(w.Y=t);
                    m=3;
                    return (a+d*t).LengthSquared(); 
                }
            }
            return(-1);
        }


        public static float ProjectOrigin(ref Vector3 a,
            ref Vector3 b,
            ref Vector3 c,
            ref Vector4 w,ref uint m)
        {
            uint[] imd3 = {1,2,0};
            Vector3[] vt  = {a,b,c};
            Vector3[] dl = {a-b,b-c,c-a};
            Vector3	n= Vector3.Cross(dl[0],dl[1]);
            float l=n.LengthSquared();
            if (l > GjkEpaSolver2.GJK_SIMPLEX3_EPS)
            {
                float mindist=-1f;
                Vector4 subw = new Vector4();
                uint subm = 0;
                for(int i=0;i<3;++i)
                {
                    if(Vector3.Dot(vt[i],Vector3.Cross(dl[i],n))>0)
                    {
                        uint j = imd3[i];
                        float subd = GJK.ProjectOrigin(ref vt[i],ref vt[j],ref subw,ref subm);
                        if((mindist<0)||(subd<mindist))
                        {
                            mindist	= subd;
                            m =	(uint)((((subm&1) != 0)?1<<i:0)+(((subm&2)!=0)?1<<(int)j:0));

                            MathUtil.VectorComponent(ref w,(int)i, subw.X);
                            MathUtil.VectorComponent(ref w,(int)j, subw.Y);
                            MathUtil.VectorComponent(ref w,(int)imd3[j],0f);				
                        }
                    }
                }
                if(mindist<0)
                {
                    float d = Vector3.Dot(a,n);	
                    float s = (float)Math.Sqrt(l);
                    Vector3	p = n * (d/l);
                    mindist	=	p.LengthSquared();
                    m =	7;
                    w.X	= (Vector3.Cross(dl[1],b-p)).Length()/s;
                    w.Y	= (Vector3.Cross(dl[2],c-p)).Length()/s;
                    w.Z	= 1-(w.X+w.Y);
                }
                return(mindist);
            }
            return(-1);
        }

        public static float ProjectOrigin(ref Vector3 a,
            ref Vector3 b,
            ref Vector3 c,
            ref Vector3 d,
            ref Vector4 w,ref uint m)
        {
            uint[] imd3 ={1,2,0};
            Vector3[]	vt = {a,b,c,d};
            Vector3[]	dl= {a-d,b-d,c-d};
            float vl= Det(dl[0],dl[1],dl[2]);
            
            bool ng=(vl*Vector3.Dot(a,Vector3.Cross(b-c,a-b)))<=0;
            if (ng && (Math.Abs(vl) > GjkEpaSolver2.GJK_SIMPLEX4_EPS))
            {
                float mindist=-1;
                Vector4 subw = new Vector4();
                uint subm = 0;
                for(int i=0;i<3;++i)
                {
                    uint j= imd3[i];
                    float s=vl*Vector3.Dot(d,Vector3.Cross(dl[i],dl[j]));
                    if(s>0)
                    {
                        float subd=GJK.ProjectOrigin(ref vt[i],ref vt[j],ref d,ref subw,ref subm);
                        if((mindist<0)||(subd<mindist))
                        {
                            mindist		=	subd;
                            m			=	(uint)((((subm&1) != 0)?1<<i:0)+
                                (((subm&2)!=0)?1<<(int)j:0)+
                                (((subm&4)!=0)?8:0));
                            MathUtil.VectorComponent(ref w,i,subw.X);
                            MathUtil.VectorComponent(ref w,(int)j,subw.Y);
                            MathUtil.VectorComponent(ref w,(int)imd3[j],0f);
                            w.W = subw.Z;
                        }
                    }
                }
                if(mindist<0)
                {
                    mindist	=	0;
                    m		=	15;
                    w.X	=	Det(c,b,d)/vl;
                    w.Y	=	Det(a,c,d)/vl;
                    w.Z	=	Det(b,a,d)/vl;
                    w.W	=	1-(w.X+w.Y+w.Z);
                }
                return(mindist);
            }
            return(-1);
        }

        public GjkEpaSolver2MinkowskiDiff m_shape;
        public Vector3 m_ray;
        public float m_distance;
        public sSimplex[] m_simplices = new sSimplex[2];
        public sSV[] m_store = new sSV[4];
        public sSV[] m_free = new sSV[4];
        public uint m_nfree;
        public uint m_current;
        public sSimplex m_simplex = new sSimplex();
        public GJKStatus m_status;
    }


        // EPA
        public class EPA
        {
            /* Fields		*/ 
            public eStatus	m_status;
            public sSimplex	m_result;
            public Vector3	m_normal;
            public float m_depth;
            public sSV[] m_sv_store = new sSV[GjkEpaSolver2.EPA_MAX_VERTICES];
            public sFace[] m_fc_store = new sFace[GjkEpaSolver2.EPA_MAX_FACES];
            public uint m_nextsv;
            public IList<sFace> m_hull = new List<sFace>();
            public IList<sFace> m_stock = new List<sFace>();
            /* Methods		*/ 
            public EPA()
            {
                Initialize();	
            }


            public void Initialize()
            {
                m_status = eStatus.Failed;
                m_normal = Vector3.Zero;
                m_depth = 0;
                m_nextsv = 0;
                m_result = new sSimplex();

                for (int i = 0; i < m_sv_store.Length; ++i)
                {
                    m_sv_store[i] = new sSV();
                }

                for (int i = 0; i < m_fc_store.Length; ++i)
                {
                    m_fc_store[i] = new sFace();
                }

                for (int i = 0; i < GjkEpaSolver2.EPA_MAX_FACES; ++i)
                {
                    Append(m_stock, m_fc_store[GjkEpaSolver2.EPA_MAX_FACES - i - 1]);
                }
            }


            public static void Bind(sFace fa,uint ea,sFace fb,uint eb)
            {
                fa.e[ea]=(uint)eb;fa.f[ea]=fb;
                fb.e[eb]=(uint)ea;fb.f[eb]=fa;
            }
            public static void Append(IList<sFace> list,sFace face)
            {
                //face.l[0]	=	null;
                //face.l[1]	=	list[0];
                //if(list.Count > 0)
                //{
                //    list[0].l[0]=face;
                //}
                //list.root	=	face;
                //++list.count;
                list.Add(face);
            }
            public static void Remove(IList<sFace> list,sFace face)
            {
                //if(face->l[1]) face->l[1]->l[0]=face->l[0];
                //if(face->l[0]) face->l[0]->l[1]=face->l[1];
                //if(face==list.root) list.root=face->l[1];
                //--list.count;
                list.Remove(face);
            }

            private void SwapSv(sSV[] array, int a, int b)
            {
                sSV temp = array[a];
                array[a] = array[b];
                array[b] = temp;
            }

            private void SwapFloat(float[] array, int a, int b)
            {
                float temp = array[a];
                array[a] = array[b];
                array[b] = temp;
            }

        public eStatus Evaluate(GJK gjk,ref Vector3 guess)
        {
            sSimplex simplex=gjk.m_simplex;
            if((simplex.rank>1)&&gjk.EncloseOrigin())
            {
                /* Clean up				*/ 
                while(m_hull.Count > 0)
                {
                    sFace	f = m_hull[0];
                    Remove(m_hull,f);
                    Append(m_stock,f);
                }

                m_status = eStatus.Valid;
                m_nextsv = 0;
                /* Orient simplex		*/ 
                if(GJK.Det(	simplex.c[0].w-simplex.c[3].w,
                    simplex.c[1].w-simplex.c[3].w,
                    simplex.c[2].w-simplex.c[3].w)<0)
                {
                    SwapSv(simplex.c,0,1);
                    SwapFloat(simplex.p,0,1);
                }
                /* Build initial hull	*/ 
                sFace[]	tetra ={NewFace(simplex.c[0],simplex.c[1],simplex.c[2],true),
                    NewFace(simplex.c[1],simplex.c[0],simplex.c[3],true),
                    NewFace(simplex.c[2],simplex.c[1],simplex.c[3],true),
                    NewFace(simplex.c[0],simplex.c[2],simplex.c[3],true)};
                if(m_hull.Count==4)
                {
                    sFace best=FindBest();
                    sFace outer = best;
                    uint pass=0;
                    uint iterations=0;
                    Bind(tetra[0],0,tetra[1],0);
                    Bind(tetra[0],1,tetra[2],0);
                    Bind(tetra[0],2,tetra[3],0);
                    Bind(tetra[1],1,tetra[3],2);
                    Bind(tetra[1],2,tetra[2],1);
                    Bind(tetra[2],2,tetra[3],1);
                    m_status=eStatus.Valid;
                    for (; iterations < GjkEpaSolver2.EPA_MAX_ITERATIONS; ++iterations)
                    {
                        if (m_nextsv < GjkEpaSolver2.EPA_MAX_VERTICES)
                        {
                            sHorizon horizon = new sHorizon() ;
                            sSV	w = m_sv_store[m_nextsv++];
                            bool valid = true;					
                            best.pass =	(uint)(++pass);
                            gjk.GetSupport(ref best.n,ref w);
                            float wdist=Vector3.Dot(best.n,w.w)-best.d;
                            if (wdist > GjkEpaSolver2.EPA_ACCURACY)
                            {
                                for(int j=0;(j<3)&&valid;++j)
                                {
                                    valid&=Expand(	pass,w,
                                        best.f[j],best.e[j],
                                        horizon);
                                }
                                if(valid&&(horizon.nf>=3))
                                {
                                    Bind(horizon.cf,1,horizon.ff,2);
                                    Remove(m_hull,best);
                                    Append(m_stock,best);
                                    best=FindBest();
                                    if (best.p >= outer.p)
                                    {
                                        outer = best;
                                    }
                                } 
                                else 
                                { 
                                    m_status=eStatus.InvalidHull;
                                    break; 
                                }
                            } 
                            else 
                            { 
                                m_status=eStatus.AccuraryReached;
                                break; 
                            }
                        } 
                        else 
                        { 
                            m_status=eStatus.OutOfVertices;
                            break; 
                        }
                    }
                    Vector3	projection=outer.n*outer.d;
                    m_normal	=	outer.n;
                    m_depth		=	outer.d;
                    m_result.rank	=	3;
                    m_result.c[0]	=	outer.c[0];
                    m_result.c[1]	=	outer.c[1];
                    m_result.c[2]	=	outer.c[2];
                    m_result.p[0]	=	Vector3.Cross(	outer.c[1].w-projection,
                        outer.c[2].w-projection).Length();
                    m_result.p[1] = Vector3.Cross(outer.c[2].w - projection,
                        outer.c[0].w-projection).Length();
                    m_result.p[2] = Vector3.Cross(outer.c[0].w - projection,
                        outer.c[1].w-projection).Length();
                    float sum=m_result.p[0]+m_result.p[1]+m_result.p[2];
                    m_result.p[0]	/=	sum;
                    m_result.p[1]	/=	sum;
                    m_result.p[2]	/=	sum;
                    return(m_status);
                }
            }
            /* Fallback		*/ 
            m_status	=	eStatus.FallBack;
            m_normal	=	-guess;
            float nl=m_normal.LengthSquared();
            if(nl>0)
            {
                m_normal.Normalize();
            }
            else
            {
                m_normal = new Vector3(1,0,0);
            }

            m_depth	=	0;
            m_result.rank=1;
            m_result.c[0]=simplex.c[0];
            m_result.p[0]=1;	
            return(m_status);
        }

        public sFace NewFace(sSV a,sSV b,sSV c,bool forced)
        {
            if(m_stock.Count > 0)
            {
                sFace face=m_stock[0];
                Remove(m_stock,face);
                Append(m_hull,face);
                face.pass	=	0;
                face.c[0]	=	a;
                face.c[1]	=	b;
                face.c[2]	=	c;
                face.n		=	Vector3.Cross(b.w-a.w,c.w-a.w);
                float l=face.n.Length();
                bool v = l > GjkEpaSolver2.EPA_ACCURACY;
                face.p = Math.Min(Math.Min(
                    Vector3.Dot(a.w,Vector3.Cross(face.n,a.w-b.w)),
                    Vector3.Dot(b.w,Vector3.Cross(face.n,b.w-c.w))),
                    Vector3.Dot(c.w,Vector3.Cross(face.n,c.w-a.w)))	/
                    (v?l:1);
                face.p = face.p >= -GjkEpaSolver2.EPA_INSIDE_EPS ? 0 : face.p;
                if(v)
                {
                    face.d = Vector3.Dot(a.w,face.n)/l;
                    face.n /= l;
                    if (forced || (face.d >= -GjkEpaSolver2.EPA_PLANE_EPS))
                    {
                        return(face);
                    } 
                    else 
                    {
                        m_status=eStatus.NonConvex;
                    }
                } 
                else 
                {
                    m_status=eStatus.Degenerated;
                }
                Remove(m_hull,face);
                Append(m_stock,face);
                return null;
            }
            m_status=m_stock.Count > 0?eStatus.OutOfVertices:eStatus.OutOfFaces;
            return null;
        }

        public sFace FindBest()
        {
            sFace minf = m_hull[0];
            float mind=minf.d*minf.d;
            float maxp=minf.p;

            //for(sFace f=minf.l[1];f != null;f=f.l[1])
            for (int i = 0; i < m_hull.Count;++i )
            {
                sFace f = m_hull[i];
                float sqd = f.d * f.d;
                if ((f.p >= maxp) && (sqd < mind))
                {
                    minf = f;
                    mind = sqd;
                    maxp = f.p;
                }
            }
            return minf;
        }

        public bool	Expand(uint pass,sSV w,sFace f,uint e,sHorizon horizon)
        {
            uint[]	i1m3 = {1,2,0};
            uint[]  i2m3 = {2,0,1};
            if(f.pass!=pass)
            {
                uint e1 = i1m3[e];
                if ((Vector3.Dot(f.n, w.w) - f.d) < -GjkEpaSolver2.EPA_PLANE_EPS)
                {
                    sFace nf = NewFace(f.c[e1],f.c[e],w,false);
                    if(nf != null)
                    {
                        Bind(nf,0,f,e);
                        if(horizon.cf != null)
                        {
                            Bind(horizon.cf,1,nf,2);
                        }
                        else 
                        {
                            horizon.ff=nf;
                        }
                        horizon.cf=nf;
                        ++horizon.nf;
                        return(true);
                    }
                }
                else
                {
                    uint e2=i2m3[e];
                    f.pass = (uint)pass;
                    if(	Expand(pass,w,f.f[e1],f.e[e1],horizon)&&
                        Expand(pass,w,f.f[e2],f.e[e2],horizon))
                    {
                        Remove(m_hull,f);
                        Append(m_stock,f);
                        return(true);
                    }
                }
            }
            return(false);
        }
    }
    
    public class sFace
    {
        public Vector3	n;
        public float d;
        public float p;
        public sSV[] c = new sSV[3];
        public sFace[] f = new sFace[3];
        //public sFace[] l = new sFace[2];
        public uint[] e = new uint[3];
        public uint pass =0;
    }

    public class sHorizon
    {
        public sHorizon()
        {
            cf = null;
            ff = null;
            nf = 0;
        }
        public sFace cf;
        public sFace ff;
        public uint nf;
    }

        
    public enum eStatus
    {
        Valid,
        Touching,
        Degenerated,
        NonConvex,
        InvalidHull,		
        OutOfFaces,
        OutOfVertices,
        AccuraryReached,
        FallBack,
        Failed		
    }

}


