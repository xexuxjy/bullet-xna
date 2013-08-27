using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;

namespace BulletXNA.LinearMath
{

    public class PooledType<T> where T : new()
    {

        public virtual T Get()
        {
            if (m_pool.Count == 0)
            {
                m_pool.Push(new T());
            }
            return m_pool.Pop();
        }

        public virtual void Free(T obj)
        {
            Debug.Assert(!m_pool.Contains(obj));
            m_pool.Push(obj);
        }


        private Stack<T> m_pool = new Stack<T>();
    }

    public class LockedPooledType<T> where T : new()
    {

        public virtual T Get()
        {
            lock (this)
            {
                if (m_pool.Count == 0)
                {
                    m_pool.Push(new T());
                }
                return m_pool.Pop();
            }
        }

        public virtual void Free(T obj)
        {
            Debug.Assert(!m_pool.Contains(obj));
            lock (this)
            {
                m_pool.Push(obj);
            }
        }


        private Stack<T> m_pool = new Stack<T>();
    }


}
