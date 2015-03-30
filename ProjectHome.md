This is a partial port of the bullet physics library (v2.79) , written in C# and using the XNA 4.0 libraries. It's aim as much as possible was to provide a fairly straight port of the c++ libraries to make maintaining compatability between the 2 version easier.

**Current stable version is : 127**


It currently supports:
  * All collision shapes
  * All constraint types

It doesn't support :

  * Bullet Serialization
  * Soft Body

It supports :

  * pc,360 and windows phone 7.1.


Links / Examples
  * Craft Studio : http://www.youtube.com/watch?v=Xc9w5ejM-ao
  * Killer Core : http://www.youtube.com/watch?v=uPpOkA-bu6c


Known Issues :
  * Ragdoll Demo / Contraints - twitchy
  * BvhBroadphase - seems to be a bug in cleanup , removing then adding all items breaks collision - noticed in KillerCore

To Do
  * **2.82 updates where appropriate.**
  * Mult-threading support  (see multi-threaded branch)
  * MonoGame support
  * **Remove dependencies on 'core' xna where possible to move to a more platform netural library**
  * Add serialisation
  * Further performance improvements
    * Most widely used internal objects are now pooled.
    * Most chokepoints now have in-lined code.
  * New demos
  * Soft Body if I'm feeling brave.
  * Playstation Suite? ;)


**Many thanks to the JetBrains team for the use of their excellent [dotTrace profiling tools that have helped tremendously  on this port.](http://www.jetbrains.com/profiler/)**