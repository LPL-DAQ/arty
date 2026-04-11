target_sources(app PRIVATE
    RCSController.cpp
    RCSStateIdle.cpp
    RCSStateRollSeq.cpp
    RCSStateValveSeq.cpp
    RCSStateAbort.cpp
    hornet/RCSHornet.cpp
    ranger/RCSRanger.cpp
    )
