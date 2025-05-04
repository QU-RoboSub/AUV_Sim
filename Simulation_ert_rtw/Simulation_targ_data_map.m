    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 0;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (Simulation_P)
        ;%

            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 2;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (Simulation_B)
        ;%
            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% Simulation_B.In1
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section

            section.nData     = 15;
            section.data(15)  = dumData; %prealloc

                    ;% Simulation_B.theta
                    section.data(1).logicalSrcIdx = 1;
                    section.data(1).dtTransOffset = 0;

                    ;% Simulation_B.nu
                    section.data(2).logicalSrcIdx = 2;
                    section.data(2).dtTransOffset = 3;

                    ;% Simulation_B.Add
                    section.data(3).logicalSrcIdx = 3;
                    section.data(3).dtTransOffset = 9;

                    ;% Simulation_B.Add1
                    section.data(4).logicalSrcIdx = 4;
                    section.data(4).dtTransOffset = 12;

                    ;% Simulation_B.Reshape
                    section.data(5).logicalSrcIdx = 5;
                    section.data(5).dtTransOffset = 15;

                    ;% Simulation_B.ddtnu
                    section.data(6).logicalSrcIdx = 6;
                    section.data(6).dtTransOffset = 21;

                    ;% Simulation_B.p
                    section.data(7).logicalSrcIdx = 7;
                    section.data(7).dtTransOffset = 27;

                    ;% Simulation_B.ddttheta
                    section.data(8).logicalSrcIdx = 8;
                    section.data(8).dtTransOffset = 30;

                    ;% Simulation_B.dpdt
                    section.data(9).logicalSrcIdx = 9;
                    section.data(9).dtTransOffset = 33;

                    ;% Simulation_B.Gain1
                    section.data(10).logicalSrcIdx = 10;
                    section.data(10).dtTransOffset = 36;

                    ;% Simulation_B.Gain2
                    section.data(11).logicalSrcIdx = 11;
                    section.data(11).dtTransOffset = 39;

                    ;% Simulation_B.net_force
                    section.data(12).logicalSrcIdx = 12;
                    section.data(12).dtTransOffset = 42;

                    ;% Simulation_B.net_moment
                    section.data(13).logicalSrcIdx = 13;
                    section.data(13).dtTransOffset = 45;

                    ;% Simulation_B.thruster_forces
                    section.data(14).logicalSrcIdx = 14;
                    section.data(14).dtTransOffset = 48;

                    ;% Simulation_B.global_force
                    section.data(15).logicalSrcIdx = 15;
                    section.data(15).dtTransOffset = 56;

            nTotData = nTotData + section.nData;
            sigMap.sections(2) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 8;
        sectIdxOffset = 2;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (Simulation_DW)
        ;%
            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% Simulation_DW.obj
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% Simulation_DW.obj_g
                    section.data(1).logicalSrcIdx = 1;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 4;
            section.data(4)  = dumData; %prealloc

                    ;% Simulation_DW.Scope_PWORK.LoggedData
                    section.data(1).logicalSrcIdx = 2;
                    section.data(1).dtTransOffset = 0;

                    ;% Simulation_DW.Scope2_PWORK.LoggedData
                    section.data(2).logicalSrcIdx = 3;
                    section.data(2).dtTransOffset = 6;

                    ;% Simulation_DW.Scope1_PWORK.LoggedData
                    section.data(3).logicalSrcIdx = 4;
                    section.data(3).dtTransOffset = 12;

                    ;% Simulation_DW.Scope3_PWORK.LoggedData
                    section.data(4).logicalSrcIdx = 5;
                    section.data(4).dtTransOffset = 18;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% Simulation_DW.sfEvent
                    section.data(1).logicalSrcIdx = 6;
                    section.data(1).dtTransOffset = 0;

                    ;% Simulation_DW.sfEvent_a
                    section.data(2).logicalSrcIdx = 7;
                    section.data(2).dtTransOffset = 1;

                    ;% Simulation_DW.sfEvent_m
                    section.data(3).logicalSrcIdx = 8;
                    section.data(3).dtTransOffset = 2;

                    ;% Simulation_DW.sfEvent_o
                    section.data(4).logicalSrcIdx = 9;
                    section.data(4).dtTransOffset = 3;

                    ;% Simulation_DW.sfEvent_as
                    section.data(5).logicalSrcIdx = 10;
                    section.data(5).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% Simulation_DW.EnabledSubsystem_SubsysRanBC
                    section.data(1).logicalSrcIdx = 11;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
            clear section

            section.nData     = 7;
            section.data(7)  = dumData; %prealloc

                    ;% Simulation_DW.objisempty
                    section.data(1).logicalSrcIdx = 12;
                    section.data(1).dtTransOffset = 0;

                    ;% Simulation_DW.objisempty_g
                    section.data(2).logicalSrcIdx = 13;
                    section.data(2).dtTransOffset = 1;

                    ;% Simulation_DW.doneDoubleBufferReInit
                    section.data(3).logicalSrcIdx = 14;
                    section.data(3).dtTransOffset = 2;

                    ;% Simulation_DW.doneDoubleBufferReInit_h
                    section.data(4).logicalSrcIdx = 15;
                    section.data(4).dtTransOffset = 3;

                    ;% Simulation_DW.doneDoubleBufferReInit_k
                    section.data(5).logicalSrcIdx = 16;
                    section.data(5).dtTransOffset = 4;

                    ;% Simulation_DW.doneDoubleBufferReInit_i
                    section.data(6).logicalSrcIdx = 17;
                    section.data(6).dtTransOffset = 5;

                    ;% Simulation_DW.doneDoubleBufferReInit_l
                    section.data(7).logicalSrcIdx = 18;
                    section.data(7).dtTransOffset = 6;

            nTotData = nTotData + section.nData;
            dworkMap.sections(6) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% Simulation_DW.CoreSubsys[0].sfEvent
                    section.data(1).logicalSrcIdx = 19;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(7) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% Simulation_DW.CoreSubsys[0].doneDoubleBufferReInit
                    section.data(1).logicalSrcIdx = 20;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(8) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 1106676841;
    targMap.checksum1 = 1305328841;
    targMap.checksum2 = 1809502106;
    targMap.checksum3 = 2973264343;

