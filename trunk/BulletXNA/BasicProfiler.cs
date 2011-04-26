using System;
using System.Diagnostics;

namespace BulletXNA
{
    //public class BasicProfile : IProfile
    //{

    //    #region IProfile Members

    //    public void StartProfile(string name)
    //    {
    //        throw new NotImplementedException();
    //    }

    //    public void EndProfile(string name)
    //    {
    //        throw new NotImplementedException();
    //    }

    //    #endregion
    //}

    public class BasicProfileManager : IProfileManager
    {
        public BasicProfileManager()
        {
            m_root = new CProfileNode("Root", null,this);
            m_currentNode = m_root;
            m_stopwatch = new Stopwatch();
        }

        #region IProfileManager Members

        public void Start_Profile(string name)
        {
            if (name != m_currentNode.Get_Name())
            {
                m_currentNode = m_currentNode.Get_Sub_Node(name);
            }

            m_currentNode.Call();
        }

        public void Stop_Profile()
        {
            // Return will indicate whether we should back up to our parent (we may
            // be profiling a recursive function)
            if (m_currentNode.Return())
            {
                m_currentNode = m_currentNode.Get_Parent();
            }
        }

        public void CleanupMemory()
        {
            throw new NotImplementedException();
        }

        public void Reset()
        {
            m_stopwatch.Reset();
            m_stopwatch.Start();
            m_root.Reset();
            m_root.Call();
            m_frameCounter = 0;
            m_resetTime = m_stopwatch.ElapsedTicks;
        }

        public void Increment_Frame_Counter()
        {
            m_frameCounter++;
        }

        public int Get_Frame_Count_Since_Reset()
        {
	        long time = m_stopwatch.ElapsedTicks;
	        time -= m_resetTime;
	        return (int)((float)time / Profile_Get_Tick_Rate());
        }

        public float Profile_Get_Tick_Rate()
        {
            return 1000.0f;
        
        }

        public float Get_Time_Since_Reset()
        {
            long time = m_stopwatch.ElapsedTicks;
	        time -= m_resetTime;
	        return (float)time / Profile_Get_Tick_Rate();
        }

        public void DumpRecursive(IProfileIterator profileIterator, int spacing)
        {
            //profileIterator.First();
            //if (profileIterator.Is_Done())
            //    return;

            //float accumulated_time=0,parent_time = profileIterator.Is_Root() ? CProfileManager::Get_Time_Since_Reset() : profileIterator.Get_Current_Parent_Total_Time();
            //int i;
            //int frames_since_reset = CProfileManager::Get_Frame_Count_Since_Reset();
            //for (i=0;i<spacing;i++)	printf(".");
            //printf("----------------------------------\n");
            //for (i=0;i<spacing;i++)	printf(".");
            //printf("Profiling: %s (total running time: %.3f ms) ---\n",	profileIterator.Get_Current_Parent_Name(), parent_time );
            //float totalTime = 0.f;


            //int numChildren = 0;

            //for (i = 0; !profileIterator.Is_Done(); i++,profileIterator.Next())
            //{
            //    numChildren++;
            //    float current_total_time = profileIterator.Get_Current_Total_Time();
            //    accumulated_time += current_total_time;
            //    float fraction = parent_time > SIMD_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
            //    {
            //        int i;	for (i=0;i<spacing;i++)	printf(".");
            //    }
            //    printf("%d -- %s (%.2f %%) :: %.3f ms / frame (%d calls)\n",i, profileIterator.Get_Current_Name(), fraction,(current_total_time / (double)frames_since_reset),profileIterator.Get_Current_Total_Calls());
            //    totalTime += current_total_time;
            //    //recurse into children
            //}

            //if (parent_time < accumulated_time)
            //{
            //    printf("what's wrong\n");
            //}
            //for (i=0;i<spacing;i++)	printf(".");
            //printf("%s (%.3f %%) :: %.3f ms\n", "Unaccounted:",parent_time > SIMD_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);

            //for (i=0;i<numChildren;i++)
            //{
            //    profileIterator.Enter_Child(i);
            //    dumpRecursive(profileIterator,spacing+3);
            //    profileIterator.Enter_Parent();
            //}
        }

        public void DumpAll()
        {
    //CProfileIterator* profileIterator = 0;
    //profileIterator = CProfileManager::Get_Iterator();

    //dumpRecursive(profileIterator,0);

    //CProfileManager::Release_Iterator(profileIterator);
        }

        public IProfileIterator getIterator()
        {
            return new BasicProfileIterator(m_root);
        }

        #endregion

        CProfileNode m_root;
        CProfileNode m_currentNode;
        int m_frameCounter = 0;
        long m_resetTime = 0;
        public Stopwatch m_stopwatch;
    }



    public class CProfileNode 
    {
        public CProfileNode(String name, CProfileNode parent, BasicProfileManager profileManager)
        {
            m_name = name;
            m_parent = parent;
            m_profileManager = profileManager;
            Reset();
        }

        public void Cleanup()
        {
            m_child = null;
            m_sibling = null;

        }

	    public CProfileNode Get_Sub_Node(String name )
        {
            // Try to find this sub node
            CProfileNode child = m_child;
            while (child != null)
            {
                if (child.m_name == name)
                {
                    return child;
                }
                child = child.m_sibling;
            }

            // We didn't find it, so add it

            CProfileNode node = new CProfileNode(name, this,m_profileManager);
            node.m_sibling = m_child;
            m_child = node;
            return node;

        }

	    public CProfileNode Get_Parent()		
        { 
            return m_parent; 
        }
	
        public CProfileNode Get_Sibling()		
        { 
            return m_sibling; 
        }
	    public CProfileNode Get_Child()			
        { 
            return m_child; 
        }

	    public void CleanupMemory()
        {
        }
	    
        public void Reset()
        {
            m_totalCalls = 0;
            m_totalTime = 0.0f;


            if (m_child != null)
            {
                m_child.Reset();
            }
            if (m_sibling != null)
            {
                m_sibling.Reset();
            }

        }
	
        public void	Call()
        {
            m_totalCalls++;
            if (m_recursionCounter++ == 0)
            {
                m_startTime = m_profileManager.m_stopwatch.ElapsedTicks;
            }
        }
	
        public bool	Return()
        {
	        if ( --m_recursionCounter == 0 && m_totalCalls != 0 ) 
            {
                long time = m_profileManager.m_stopwatch.ElapsedTicks;
		        time-=m_startTime;
		        m_totalTime += (float)time / m_profileManager.Profile_Get_Tick_Rate();
	        }
            return (m_recursionCounter == 0);
        }

	    public String Get_Name()				
        { 
            return m_name; 
        }
	    
        public int Get_Total_Calls()
        { 
            return m_totalCalls; 
        }
	
        public float Get_Total_Time()		
        { 
            return m_totalTime; 
        }

	    protected String m_name;
	    protected int m_totalCalls;
	    protected float m_totalTime;
        protected long m_startTime;
	    protected int m_recursionCounter;

	    protected CProfileNode m_parent;
	    protected CProfileNode m_child;
	    protected CProfileNode m_sibling;
        protected BasicProfileManager m_profileManager;
}

        public class BasicProfileIterator : IProfileIterator
{

            public BasicProfileIterator(CProfileNode start)
            {
                m_currentParent = start;
                m_currentChild = start.Get_Child();
            }

    #region IProfileIterator Members

    public void First()
    {
        m_currentChild = m_currentParent.Get_Child();
    }

    public void Next()
    {
        m_currentChild = m_currentChild.Get_Sibling();
    }

    public bool Is_Done()
    {
        return m_currentChild == null;
    }

    public bool Is_Root()
    {
        return m_currentParent.Get_Parent() == null;
    }

    public void Enter_Child(int index)
    {
        m_currentChild = m_currentParent.Get_Child();
        while ((m_currentChild != null) && (index != 0))
        {
            index--;
            m_currentChild = m_currentChild.Get_Sibling();
        }

        if (m_currentChild != null)
        {
            m_currentParent = m_currentChild;
            m_currentChild = m_currentParent.Get_Child();
        }
    }

    public void Enter_Largest_Child()
    {
        throw new NotImplementedException();
    }

    public void Enter_Parent()
    {
        if (m_currentParent.Get_Parent() != null)
        {
            m_currentParent = m_currentParent.Get_Parent();
        }
        m_currentChild = m_currentParent.Get_Child();
    }

    public string Get_Current_Name()
    {
        return m_currentChild.Get_Name();
            
    }

    public int Get_Current_Total_Calls()
    {
        return m_currentChild.Get_Total_Calls();
    }

    public float Get_Current_Total_Time()
    {
        return m_currentChild.Get_Total_Time();
    }

    public string Get_Current_Parent_Name()
    {
        return m_currentParent.Get_Name();
    }

    public int Get_Current_Parent_Total_Calls()
    {
        return m_currentParent.Get_Total_Calls();
    }

    public float Get_Current_Parent_Total_Time()
    {
        return m_currentParent.Get_Total_Time();
    }

    #endregion

    private CProfileNode m_currentParent;
    private CProfileNode m_currentChild;

}





}
