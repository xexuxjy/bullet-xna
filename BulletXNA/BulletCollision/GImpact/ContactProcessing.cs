/*
 * 
 * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
 *
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System;
using BulletXNA.LinearMath;


namespace BulletXNA.BulletCollision
{
    public class ContactProcessing
    {
    }

    public struct GIM_CONTACT
    {
        public const int NORMAL_CONTACT_AVERAGE = 1;
        public const float CONTACT_DIFF_EPSILON = 0.00001f;


        public IndexedVector3 m_point;
        public IndexedVector3 m_normal;
        public float m_depth;//Positive value indicates interpenetration
        //public float m_distance;//Padding not for use
        public int m_feature1;//Face number
        public int m_feature2;//Face number

        public GIM_CONTACT(ref GIM_CONTACT contact)
        {
            m_point = contact.m_point;
            m_normal = contact.m_normal;
            m_depth = contact.m_depth;
            m_feature1 = contact.m_feature1;
            m_feature2 = contact.m_feature2;
        }

        public GIM_CONTACT(ref IndexedVector3 point, ref IndexedVector3 normal,
                        float depth, int feature1, int feature2)
        {
            m_point = point;
            m_normal = normal;
            m_depth = depth;
            m_feature1 = feature1;
            m_feature2 = feature2;
        }

        //! Calcs key for coord classification
        public uint CalcKeyContact()
        {
            int[] _coords = new int[]{
    		(int)(m_point.X*1000.0f+1.0f),
    		(int)(m_point.Y*1333.0f),
    		(int)(m_point.Z*2133.0f+3.0f)};
            uint _hash = 0;
            uint _uitmp = (uint)_coords[0];
            _hash = _uitmp;
            _uitmp = (uint)_coords[1];
            _hash += (_uitmp) << 4;
            _uitmp = (uint)_coords[2];
            _hash += (_uitmp) << 8;
            return _hash;
        }

        public void InterpolateNormals(IndexedVector3[] normals, int normal_count)
        {
            IndexedVector3 vec_sum = m_normal;
            for (int i = 0; i < normal_count; i++)
            {
                vec_sum += normals[i];
            }

            float vec_sum_len = vec_sum.LengthSquared();
            if (vec_sum_len < CONTACT_DIFF_EPSILON) return;

            //GIM_INV_SQRT(vec_sum_len,vec_sum_len); // 1/sqrt(vec_sum_len)

            m_normal = vec_sum / (float)Math.Sqrt(vec_sum_len);
        }

    }


    public class ContactArray : ObjectArray<GIM_CONTACT>
    {
        public const int MAX_COINCIDENT = 8;

        public ContactArray()
            : base(64)
        {
        }

        void PushContact(ref IndexedVector3 point, ref IndexedVector3 normal,
            float depth, int feature1, int feature2)
        {
            Add(new GIM_CONTACT(ref point, ref normal, depth, feature1, feature2));
        }

        void PushContact(ref IndexedVector3 point, ref IndexedVector4 normal,
            float depth, int feature1, int feature2)
        {
            IndexedVector3 temp = new IndexedVector3(normal.X, normal.Y, normal.Z);
            Add(new GIM_CONTACT(ref point, ref temp, depth, feature1, feature2));
        }


        public void PushTriangleContacts(GIM_TRIANGLE_CONTACT tricontact,
            int feature1, int feature2)
        {
            for (int i = 0; i < tricontact.m_point_count; i++)
            {
                PushContact(
                    ref tricontact.m_points[i],
                    ref tricontact.m_separating_normal,
                    tricontact.m_penetration_depth, feature1, feature2);
            }
        }

        public void MergeContacts(ContactArray contacts)
        {
            MergeContacts(contacts, true);
        }

        public void MergeContacts(ContactArray contacts, bool normal_contact_average)
        {
            Clear();

            if (contacts.Count == 0) return;


            if (contacts.Count == 1)
            {
                Add(contacts[0]);
                return;
            }

            ObjectArray<CONTACT_KEY_TOKEN> keycontacts = new ObjectArray<CONTACT_KEY_TOKEN>();

            keycontacts.Capacity = contacts.Count;

            //fill key contacts

            for (int i = 0; i < contacts.Count; i++)
            {
                keycontacts.Add(new CONTACT_KEY_TOKEN(contacts[i].CalcKeyContact(), i));
            }

            //sort keys

            keycontacts.Sort(new Comparison<CONTACT_KEY_TOKEN>(CONTACT_KEY_TOKEN.SortPredicate));


            // Merge contacts
            int coincident_count = 0;
            IndexedVector3[] coincident_normals = new IndexedVector3[MAX_COINCIDENT];

            uint last_key = keycontacts[0].m_key;
            uint key = 0;

            Add(contacts[keycontacts[0].m_value]);

            GIM_CONTACT pcontact = this[0];
            int pcontactIndex = 0;

            for (int i = 1; i < keycontacts.Count; i++)
            {
                key = keycontacts[i].m_key;
                GIM_CONTACT scontact = contacts[keycontacts[i].m_value];

                if (last_key == key)//same points
                {
                    //merge contact
                    if (pcontact.m_depth - GIM_CONTACT.CONTACT_DIFF_EPSILON > scontact.m_depth)//)
                    {
                        //*pcontact = *scontact;
                        this[pcontactIndex] = scontact;
                        coincident_count = 0;
                    }
                    else if (normal_contact_average)
                    {
                        if (Math.Abs(pcontact.m_depth - scontact.m_depth) < GIM_CONTACT.CONTACT_DIFF_EPSILON)
                        {
                            if (coincident_count < MAX_COINCIDENT)
                            {
                                coincident_normals[coincident_count] = scontact.m_normal;
                                coincident_count++;
                            }
                        }
                    }
                }
                else
                {//add new contact

                    if (normal_contact_average && coincident_count > 0)
                    {
                        pcontact.InterpolateNormals(coincident_normals, coincident_count);
                        coincident_count = 0;
                    }

                    Add(scontact);
                    pcontactIndex = Count - 1;
                    pcontact = this[pcontactIndex];
                }
                last_key = key;
            }

        }

        public void MergeContactsUnique(ContactArray contacts)
        {
            Clear();

            if (contacts.Count == 0) return;

            if (contacts.Count == 1)
            {
                Add(contacts[0]);
                return;
            }

            GIM_CONTACT average_contact = contacts[0];

            for (int i = 1; i < contacts.Count; i++)
            {
                average_contact.m_point += contacts[i].m_point;
                average_contact.m_normal += contacts[i].m_normal * contacts[i].m_depth;
            }

            // can't see how this does anything....
            //divide
            float divide_average = 1.0f / ((float)contacts.Count);

            average_contact.m_point *= divide_average;

            average_contact.m_normal *= divide_average;

            average_contact.m_depth = average_contact.m_normal.Length();

            average_contact.m_normal /= average_contact.m_depth;


        }
    }


    public class CONTACT_KEY_TOKEN
    {
        public uint m_key;
        public int m_value;
        public CONTACT_KEY_TOKEN()
        {
        }

        public CONTACT_KEY_TOKEN(uint key, int token)
        {
            m_key = key;
            m_value = token;
        }

        public CONTACT_KEY_TOKEN(CONTACT_KEY_TOKEN rtoken)
        {
            m_key = rtoken.m_key;
            m_value = rtoken.m_value;
        }

        public static int SortPredicate(CONTACT_KEY_TOKEN lhs, CONTACT_KEY_TOKEN rhs)
        {

            return lhs.m_key < rhs.m_key ? 1 : -1;
        }

        //public bool operator <(CONTACT_KEY_TOKEN other)
        //{
        //    return (m_key < other.m_key);
        //}

        //public bool operator >(CONTACT_KEY_TOKEN other)
        //{
        //    return (m_key > other.m_key);
        //}

    };

    //class CONTACT_KEY_TOKEN_COMP
    //{
    //    public:

    //        bool operator() ( const CONTACT_KEY_TOKEN& a, const CONTACT_KEY_TOKEN& b )
    //        {
    //            return ( a < b );
    //        }
    //};


}
