/*-----------------------------------------------------------------
  OD for MN, CN1, CN2

  This OBD is configured for a network of one MN and 2 CNs with node ID respectively 1 and 2.
  MN writes output variables of CNs on indexes 6200 and 6203
  bVarOut1_l is the output variable defined into the demo_main source files of CN1 and MN linked to 6200/01 index/subindex
  bVarOut2_l is the output variable defined into the demo_main source files of CN2 and MN linked to 6203/01 index/subindex
  bVarIn1_l is the input variable defined into  the demo_main source files of CN1 and MN linked to 6000/01 index/subindex
  bVarIn2_l is the input variable defined into  the demo_main source files of CN2 and MN linked to 6003/01 index/subindex

  CN1 writes bVarIn1_l and reads bVarOut1_l
  CN2 writes bVarIn2_l and reads bVarOut2_l
  MN writes bVarOut1_l and bVarOut2_l, and it reads bVarIn1_l and bVarIn2_l

	Authors
   Trubia Massimo, Tognazzolo Alessio
//-----------------------------------------------------------------*/
#define EPL_OBD_DEFINE_MACRO
    #include "EplObdMacro.h"
#undef EPL_OBD_DEFINE_MACRO

EPL_OBD_BEGIN ()

    EPL_OBD_BEGIN_PART_GENERIC ()

        #include "../Generic/objdict_1000-13ff.h"

//MN
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) != 0)

		//MN RPDOs

        // Object 1400h: PDO_RxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1400, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1400)

        // Object 1401h: PDO_RxCommParam_01h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1401, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1401, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1401)


        // Object 1600h: PDO_RxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1600, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1600)

        // Object 1601h: PDO_RxMappParam_01h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1601, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016003LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1601, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1601)



		//MN TPDOs

        // Object 1800h: PDO_TxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1800, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1800)

        // Object 1801h: PDO_TxCommParam_01h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1801, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1801, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1801, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1801, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1801)

        // Object 1A00h: PDO_TxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A00, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016200LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1A00)

        // Object 1A01h: PDO_TxMappParam_01h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A01, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A01, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016203LL)
        EPL_OBD_END_INDEX(0x1A01)

#endif


//CNs
#if (((EPL_MODULE_INTEGRATION) & (EPL_MODULE_NMT_MN)) == 0 )

//CN1
#ifdef CN1
        // Object 1400h: PDO_RxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1400, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1400)

        // Object 1600h: PDO_RxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1600, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016200LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1600)

        // Object 1800h: PDO_TxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1800, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1800)

        // Object 1A00h: PDO_TxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A00, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016000LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1A00)

#endif

//CN2
#ifdef CN2
        // Object 1400h: PDO_RxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1400, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1400, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1400)

        // Object 1600h: PDO_RxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1600, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016203LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1600, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1600)

  // Object 1800h: PDO_TxCommParam_00h_REC
        EPL_OBD_BEGIN_INDEX_RAM(0x1800, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, NumberOfEntries, 0x02)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x01, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NodeID_U8, 0x00)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1800, 0x02, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, MappingVersion_U8, 0x00)
        EPL_OBD_END_INDEX(0x1800)

        // Object 1A00h: PDO_TxMappParam_00h_AU64
        EPL_OBD_BEGIN_INDEX_RAM(0x1A00, 0x03, EplPdouCbObdAccess)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x00, kEplObdTypUInt8, kEplObdAccRW, tEplObdUnsigned8, NumberOfEntries, 0x01)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x01, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x0008000000016003LL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x1A00, 0x02, kEplObdTypUInt64, kEplObdAccRW, tEplObdUnsigned64, ObjectMapping, 0x00)
        EPL_OBD_END_INDEX(0x1A00)


#endif


#endif


        #include "../Generic/objdict_1b00-1fff.h"

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_MANUFACTURER ()

    EPL_OBD_END_PART ()

    EPL_OBD_BEGIN_PART_DEVICE ()


        EPL_OBD_BEGIN_INDEX_RAM(0x6000, 0x02, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6000, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, number_of_entries, 0x1)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6000, 0x01, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Sendb1, 0x0)
        EPL_OBD_END_INDEX(0x6000)

        EPL_OBD_BEGIN_INDEX_RAM(0x6003, 0x02, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6003, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, number_of_entries, 0x1)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6003, 0x01, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Sendb1, 0x0)
        EPL_OBD_END_INDEX(0x6003)

        EPL_OBD_BEGIN_INDEX_RAM(0x6100, 0x01, NULL)
            EPL_OBD_SUBINDEX_RAM_DOMAIN(0x6100,0x00,kEplObdAccRW,TEST_DOMAIN)
        EPL_OBD_END_INDEX(0x6100)

        EPL_OBD_BEGIN_INDEX_RAM(0x6200, 0x02, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6200, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, number_of_entries, 0x1)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6200, 0x01, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Sendb1, 0x0)
        EPL_OBD_END_INDEX(0x6200)

		EPL_OBD_BEGIN_INDEX_RAM(0x6203, 0x02, NULL)
            EPL_OBD_SUBINDEX_RAM_VAR(0x6203, 0x00, kEplObdTypUInt8, kEplObdAccConst, tEplObdUnsigned8, number_of_entries, 0x1)
            EPL_OBD_SUBINDEX_RAM_USERDEF(0x6203, 0x01, kEplObdTypUInt8, kEplObdAccVPRW, tEplObdUnsigned8, Sendb1, 0x0)
        EPL_OBD_END_INDEX(0x6203)


    EPL_OBD_END_PART ()

EPL_OBD_END ()

#define EPL_OBD_UNDEFINE_MACRO
    #include "EplObdMacro.h"
#undef EPL_OBD_UNDEFINE_MACRO
