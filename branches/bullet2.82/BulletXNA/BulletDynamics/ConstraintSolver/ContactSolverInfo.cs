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

namespace BulletXNA.BulletDynamics
{
    public class ContactSolverInfoData
    {
        public float m_tau;
		public float m_damping; //global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
        public float m_friction;
        public float m_timeStep;
        public float m_restitution;
        public int m_numIterations;
        public float m_maxErrorReduction;
        public float m_sor;
        public float m_erp;//used as Baumgarte factor
        public float m_erp2;//used in Split Impulse
        public float m_globalCfm;//constraint force mixing
        public bool m_splitImpulse;
        public float m_splitImpulsePenetrationThreshold;
        public float m_linearSlop;
        public float m_warmstartingFactor;

        public SolverMode m_solverMode;
        public int m_restingContactRestitutionThreshold;
        public int m_minimumSolverBatchSize;

    }

    public class ContactSolverInfo : ContactSolverInfoData
    {
	    public ContactSolverInfo()
	    {
		    m_tau = 0.6f;
		    m_damping = 1.0f;
		    m_friction = 0.3f;
		    m_restitution = 0f;
		    m_maxErrorReduction = 20f;
		    m_numIterations = 10;
		    m_erp = 0.2f;
		    m_erp2 = 0.1f;
		    m_globalCfm = 0f;
            m_sor = 1f;
		    m_splitImpulse = false;
		    m_splitImpulsePenetrationThreshold = -0.02f;
            m_linearSlop = 0f;
		    m_warmstartingFactor=0.85f;
            m_solverMode = SolverMode.SOLVER_USE_WARMSTARTING | SolverMode.SOLVER_SIMD;
		    m_restingContactRestitutionThreshold = 2;//resting contact lifetime threshold to disable restitution
            m_minimumSolverBatchSize = 128; //try to combine islands until the amount of constraints reaches this limit
            //m_minimumSolverBatchSize = 1;
	    }
    }
    [Flags]
    public enum SolverMode
    {
        SOLVER_RANDMIZE_ORDER = 1,
        SOLVER_FRICTION_SEPARATE = 2,
        SOLVER_USE_WARMSTARTING = 4,
        SOLVER_USE_FRICTION_WARMSTARTING = 8,
        SOLVER_USE_2_FRICTION_DIRECTIONS = 16,
        SOLVER_ENABLE_FRICTION_DIRECTION_CACHING = 32,
        SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION = 64,
        SOLVER_CACHE_FRIENDLY = 128,
        SOLVER_SIMD = 256,	//enabled for Windows, the solver innerloop is branchless SIMD, 40% faster than FPU/scalar version
        SOLVER_CUDA = 512	//will be open sourced during Game Developers Conference 2009. Much faster.
    }
}
