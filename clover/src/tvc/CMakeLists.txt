target_sources(app PRIVATE
    TVCController.cpp
    TVCStateIdle.cpp
    TVCStateFlight.cpp
    TVCStateSeq.cpp
    TVCStateAbort.cpp
    hornet/TVCHornet.cpp
    ranger/TVCRanger.cpp
    )
