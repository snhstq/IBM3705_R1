//*********************************************************************
//      INPUT register definitions
//*********************************************************************
#define NOTUSED    0xAA55       // Notused Ereg marker

//***   Type 2 Scanner ***
#define CMBARIN    0x40         // Interface address.
//                 0x41         // Unused.
//                 0x42         // Unused.
#define CMERREG    0x43         // Check register.
#define CMICWB0F   0x44         // ICW input register 0-15.
#define CMICWLPS   0x45         // ICW inp-ut register 16-31.
#define CMICWDPS   0x46         // Display register.
#define CMICWB32   0x47         // ICW input register 32-45.

//***   Type 3 Scanner ***
#define CMBAR      0x40         // Interface address
#define CMHSPSL    0x41         // High speed select
#define CMCKRGO    0x42         // DBAR/Check register 0
#define CMERREG    0x43         // Check register 1
#define CMICWB0F   0x44         // ICW byte 0 and PDF array
#define CMICWLPS   0x45         // ICW bytes 2 and 3-LCD/PCF/SDF
#define CMICWDSP   0x46         // Display register
#define CMICWB32   0x47         // ICW bytes 4 and 5
#define CMCSCTL    0x48         // ICW bytes 6 and 7-Cycle steal address
//                                 (byte X), control, and count
#define CMCSADR    0x49         // ICW bytes 8 and 9-Cycle steal address
//                                 bytes 0 and 1.
#define CMBCC      0x4A         // ICW bytes 10 and 11-BCC
#define CMEPCF     0x48         // ICW byte 16-Extended PCF
#define CMPDFARV   0x4C         // PDF array data bits 0-10
#define CMMCTL     0x4E         // ICW bytes 12 and 13-PDF array control
#define CMSTATUS   0x4F         // ICW bytes 14 and 15-Status bytes

//***   Type 2/3 CA ***
#define CAICWAR    0x50         // INCWAR
#define CAOCWAR    0x51         // OUTCWAR
#define CARCNT     0x52         // Control word byte count.
#define CASENSE    0x53         // Sense register and cycle steal address
//                                 register byte X.
#define CASTAT     0x54         // Status register.
#define CACTL      0x55         // Control register.
#define CACHK      0x56         // Check register.
//                 0x57            Unused.
#define CADGBSOI   0x58         // Channel bus out diagnostic register.
#define CADGCSAR   0x59         // Cycle steal address reg bytes 0 and 1
#define CADGDTBF   0x5A         // Channel adapter data buffer.
#define CADTGRI    0x5B         // Channel tag diagnostic register.
#define CADGCMDR   0x5C         // Command register.
//                 0x5D            Unused.
//                 0x5E            Unused.

//***   Type 1/4 CA ***            Inputs
#define CAISC      0x60         // Initial selection control.
#define CAISD      0x61         // Initial selection address and command.
#define CASSC      0x62         // Data/status control.
#define CASSA      0x63         // CA Address and ESC status.
#define CASD12     0x64         // Data buffer bytes 1 and 2.
#define CASD34     0x65         // Data buffer bytes 3 and 4.
#define CARNSTAT   0x66         // CA NSC status byte.
#define CAECR      0x67         // CA controls.

#define SYSXR6C    0x6C         // Extended buffer/cycle steal control
//                                 (type 4 CA)
#define SVSXR6D    0x6D         // Extended buffer/cycle steal data buffer
//                                 (type 4 CA)
#define SYSXR6E    0x6E         // Cycle steal error register and byte X
//                                 (type 4 CA)
#define SYSXR6F    0x6F         // Cycle steal addr register bytes 0 and 1
//                                 (type 4 CA)
//***   C C U  ***
#define SYSSTSZ    0x70         // Storage size installed.
#define SYSADRDT   0x71         // Panel address/data bits.
#define SYSFNINS   0x72         // Panel display function select switch
//                                 controls.
#define SYSINKEY   0x73         // Insert storage protection key.
#define SYSLAR     0x74         // Lagging address register ILAR I.
#define SYSADPG1   0x76         // Adapter level 1 interrupt request.
#define SYSADPG2   0x77         // Adapter level 2 or 3 interrupt request.
#define SYSUTILI   0x79         // Utility.
#define SYSCUCI    0x7A         // Cycle utilization counter 
#define SYSBSCRC   0x7B         // BSC CRC register.
//                 0x7C            SDLC CRC register
#define SYSMCHK    0x7D         // CCU check register.
#define SYSCCUG1   0x7E         // CCU level 1 interrupt request.
#define SYSCCUG2   0x7F         // CCU level 2,3, or 4 interrupt requests.

//*********************************************************************
//      OUTPUT register definitions
//*********************************************************************

//***   Type 2 Scanner ***
#define CMBAROUT   0x40         // Interface address.
#define CMADRSUB   0x41         // Address substitution control.
#define CMSCANLT   0x42         // Upper scan limit control.
#define CMCTL      0x43         // Control.
#define CMICWB0F   0x44         // ICW 0-15.
#define CMICWLP    0x45         // ICW 16-23.
#define CMICWS     0x46         // ICW 24-33,44.
#define CMICWB34   0x47         // ICW 34-43.

//***   Type 3 Scanner ***
#define CMBAR      0x40         // ABAR loader (interface address)
#define CMADRSUB   0x41         // Subs. control and high speed select
#define CMSCANLM   0x42         // DBAR/Scan limits
#define CMCTL      0x43         // Control
#define CMICWBOF   0x44         // SCF/PDF
#define CMICWLP    0x45         // LCO/PCF/EPCF
#define CMICWS     0x46         // SDF
#define CMICWB34   0x47         // Miscellaneous ICW bits
#define CMCSCTL    0x48         // Cycle steal address register (byte X),
                                // control, and byte count
#define CMCSADR    0x49         // Cycle steal addr register bytes 0 and 1.
#define CMBCC      0x4A         // Block check character (BCC)
#define CMPDFARY   0x4C         // PDF array data
#define CMAC       0x40         // ICW cycle steal PDFs data
#define CMMCTL     0x4E         // Cycle steal/PDF pointers-ICW control
#define CMSTATUS   0x4F         // Status bytes

//***   Type 2/3 CA ***
#define CAICWAR    0x50         // INCWAR.
#define CAOCWAR    0x51         // OUTCWAR.
#define CASENSE    0x53         // Sense register.
#define CASTAT     0x54         // Status register.
#define CACTL      0x55         // Control register.
#define CACHK      0x56         // Reset control register bits.
#define CAMODE     0x57         // Channel adapter mode register.
#define CADGBSOO   0x58         // Channel bus out diagnostic register.
//                 0x59         // Diagnostic busy (type 3 CA)
#define CADGDTBF   0x5A         // Channel adapter data buffer.
#define CADTGRO    0x58         // Channel tag diagnostic register.
                       
//***   Type 1/4 CA ***            Outputs
//                 0x60         // Reset initial selection.
//                 0x61         // Unused
//                 0x62         // Unused
#define CASSC      0x62         // Data/status control
#define CASSA      0x63         // Address and ESC status.
#define CASD12     0x64         // Data buffer bytes 1 and 2.
#define CASD34     0x65         // Data buffer bytes 3 and 4.
#define CARNSTAT   0x66         // NSC status byte.
#define CAECR      0x67         // Control.

#define SYSXR6C    0x6C         // Extended buffer/cycle steal control
                                // (type 4 CA)
#define SYSXR6D    0x6D         // Extended buffer/cycle steal data buffer
                                // (type 4 CA)
#define SYSXR6E    0x6E         // Cycle steal addr register byte X
                                // (type 4 CA)
#define SYSXR6F    0x6F         // Cycle steal addr register bytes 0 and 1
                                // (type 4 CA)

//***   C C U  ***
#define SYSHDSTP   0x70         // Hard stop.
#define SYSDSPR1   0x71         // Display register 1
#define SYSDSPR2   0x72         // Display register 2
#define SYSPRKEY   0x73         // Set system protection key
#define SYSSTKEY   0x73         // Set system storage key
#define SYSMSCTL   0x77         // Miscellaneous Control.
#define SYSDIAG    0x78         // Force CCU checks.
#define SYSUTILO   0x79         // Utility.
#define SYSCUCO    0x7A         // Cycle utilization counter reset
#define SYSPCI3    0x7C         // Set PCI L3.
#define SYSPCI4    0x7D         // Set PCI L4.
#define SYSSTMSK   0x7E         // Set mask bits.
#define SYSRSMSK   0x7F         // Reset mask bits.

