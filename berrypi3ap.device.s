; ================================================================
; berrypi3ap.device
; USB 2.0 Host Controller Driver for AmigaOS / Poseidon
;
; Target platform:
;   PiStorm / Emu68
;   Raspberry Pi 3A+ / 3B+ (BCM2837 DWC2 USB controller)
;
; Implements the Poseidon HCD ABI through the standard Exec device
; interface. The current development version includes virtual root-hub
; emulation plus control, interrupt and bulk transfer support for DWC2.
;
; Project / idea: MichiB210
; Development:    MichiB210 + Ki+Ki
; Version:        USB74 / berrypi3ap.device 74.0
;
; Experimental development driver.
; ================================================================
RT_MATCHWORD        equ $4AFC
RTF_AUTOINIT        equ $80
NT_DEVICE           equ 3

; Exec LVOs
AllocMem            equ -198
OldOpenLibrary      equ -408
CachePreDMA         equ -762
CachePostDMA        equ -768
ReplyMsg            equ -378
Forbid              equ -132
Permit              equ -138

; dos.library LVO
Delay               equ -198

; Device / Library offsets
LN_TYPE             equ 8
LN_PRI              equ 9
LN_NAME             equ 10
LIB_FLAGS           equ 14
LIB_VERSION         equ 20
LIB_REVISION        equ 22
LIB_IDSTRING        equ 24
LIB_OPENCNT         equ 32
DEV_SYSBASE         equ 34
DEV_SIZE            equ 40

; IORequest / USBIORequest offsets, pack(2)
IO_DEVICE           equ 20
IO_UNIT             equ 24
IO_COMMAND          equ 28
IO_FLAGS            equ 30
IO_ERROR            equ 31

UIO_FLAGS           equ 32
UIO_STATE           equ 34
UIO_DIRECTION       equ 36
UIO_VIRTUAL_ADDRESS equ 38
UIO_ENDPOINT        equ 40
UIO_RESERVED1       equ 42
UIO_ACTUAL_LENGTH   equ 44
UIO_DATA_BUF_LEN    equ 48
UIO_DATA_BUFFER     equ 52
UIO_RESERVED2       equ 56
UIO_TIMEOUT         equ 58
UIO_SETUP           equ 62
UIO_EXTERROR        equ 74
UIO_SPLIT_HUB_ADDR  equ 78
UIO_SPLIT_HUB_PORT  equ 80
UIO_DRIVERPRIVATE1  equ 82
UIO_DRIVERPRIVATE2  equ 86
UIO_SIZE            equ 90

IOERR_NOCMD         equ -3
IOERR_ABORTED       equ -2

CMD_RESET           equ 1
CMD_STOP            equ 6
CMD_START           equ 7
CMD_FLUSH           equ 8
CMD_DEVICE_QUERY    equ 9
CMD_USB_RESET       equ 10
CMD_USB_RESUME      equ 11
CMD_REQUEST_CONTROL equ 12
CMD_REQUEST_INTERRUPT equ 14
CMD_REQUEST_BULK    equ 15

UHDIR_SETUP         equ 0
DIRECTION_OUT       equ 1
DIRECTION_IN        equ 2

ERR_NO_ERROR        equ 0
ERR_HCI_ERROR       equ 3
ERR_STALL           equ 4
ERR_BAD_PARAMETERS  equ 11
ERR_TIMEOUT         equ 6
ERR_NAK_TIMEOUT     equ 10

UHFF_LOWSPEED       equ $0001
UHFF_NAKTIMEOUT     equ $0008
UHFF_SPLITTRANS     equ $0020
HUB_IDLE_DELAY_TICKS equ 5      ; conservative cooperative delay (~100 ms PAL)
HID_IDLE_DELAY_TICKS equ 1      ; one Exec tick between empty HID polls
HCCHAR_CHDIS           equ $40000000

TAG_DONE            equ 0
TAG_DRIVER_STATE    equ $80004712
TAG_DEVICE_VENDOR   equ $80004721
TAG_DEVICE_PRODUCT  equ $80004722
TAG_DEVICE_VERSION  equ $80004723
TAG_DEVICE_REVISION equ $80004724
TAG_DRIVER_DESC     equ $80004725
TAG_DRIVER_LICENSE  equ $80004726
TAG_DRIVER_VERSION  equ $80004731
TAG_DRIVER_FEATURES equ $80004732

DRIVER_STATE_OPERATIONAL equ 1
DRIVER_FEAT_USB2         equ 1
DRIVER_FEAT_QUICK_IO     equ 8
; USB57 deliberately does NOT advertise QUICKIO. The current synchronous
; DWC2 implementation must not be called from interrupt context.
DRIVER_FEATURES          equ DRIVER_FEAT_USB2
UHSF_OPERATIONAL         equ 1
UHSF_RESUMING            equ 2
UHSF_SUSPENDED           equ 4
UHSF_RESET               equ 8
UHFF_HIGHSPEED           equ 2

MEMF_FASTCLEAR      equ $00010004
DMA_ReadFromRAM     equ $00000008

; ------------------------------------------------
; Pi3 DWC2 / mailbox mapping under Emu68
; ------------------------------------------------
MBOX_READ       equ $F200B880
MBOX_STATUS     equ $F200B898
MBOX_WRITE      equ $F200B8A0

USB_BASE        equ $F2980000

GAHBCFG         equ USB_BASE+$008
GUSBCFG         equ USB_BASE+$00C
GRSTCTL         equ USB_BASE+$010
GINTSTS         equ USB_BASE+$014
GINTMSK         equ USB_BASE+$018
GRXFSIZ         equ USB_BASE+$024
GNPTXFSIZ       equ USB_BASE+$028
GSNPSID         equ USB_BASE+$040
HPTXFSIZ        equ USB_BASE+$100

HCFG            equ USB_BASE+$400
HFIR            equ USB_BASE+$404
HFNUM           equ USB_BASE+$408
HPRT0           equ USB_BASE+$440

HCCHAR0         equ USB_BASE+$500
HCSPLT0         equ USB_BASE+$504
HCINT0          equ USB_BASE+$508
HCINTMSK0       equ USB_BASE+$50C
HCTSIZ0         equ USB_BASE+$510
HCDMA0          equ USB_BASE+$514
USB_POWER       equ USB_BASE+$E00

GAHBCFG_DMA_EN          equ $00000020
GAHBCFG_WAIT_AXI        equ $00000010

GRSTCTL_CSFTRST         equ $00000001
GRSTCTL_RXFFLSH         equ $00000010
GRSTCTL_TXFFLSH         equ $00000020
GRSTCTL_TXFNUM_ALL      equ $00000400
GRSTCTL_AHBIDLE         equ $80000000

HPRT_CONN               equ $00000001
HPRT_CONNDET            equ $00000002
HPRT_ENA                equ $00000004
HPRT_ENACHG             equ $00000008
HPRT_OVRCHG             equ $00000020
HPRT_SPD_MASK           equ $00060000
HPRT_SPD_FULL           equ $00020000
HPRT_SPD_LOW            equ $00040000
HPRT_RST                equ $00000100
HPRT_PWR                equ $00001000
HPRT_SAFE_MASK          equ $FFFFFFD1
HPRT_SAFE_NORST         equ $FFFFFED1

; USB 2.0 hub port feature selectors
RH_PORT_ENABLE          equ 1
RH_PORT_RESET           equ 4
RH_PORT_POWER           equ 8
RH_C_PORT_CONNECTION    equ 16
RH_C_PORT_ENABLE        equ 17
RH_C_PORT_SUSPEND       equ 18
RH_C_PORT_OVERCURRENT   equ 19
RH_C_PORT_RESET         equ 20

HCCHAR_MPS_64           equ $00000040
HCCHAR_EPDIR_IN         equ $00008000
HCCHAR_LSDEV            equ $00020000
HCCHAR_MULTICNT_1       equ $00100000
HCCHAR_MULTICNT_3       equ $00300000
HCCHAR_EPTYPE_BULK       equ $00080000
HCCHAR_EPTYPE_INTR       equ $000C0000
HCCHAR_ODDFRM           equ $20000000
HCCHAR_CHENA             equ $80000000

HCSPLT_PRTADDR_MASK      equ $0000007F
HCSPLT_HUBADDR_SHIFT     equ 7
HCSPLT_XACTPOS_ALL       equ $0000C000
HCSPLT_COMPSPLT          equ $00010000
HCSPLT_SPLTENA           equ $80000000

HCINT_XFERCOMP          equ $00000001
HCINT_CHHLTD            equ $00000002
HCINT_ERROR_MASK        equ $0000078C
HCINT_ACK               equ $00000020
HCINT_NYET              equ $00000040

HCTSIZ_SETUP_8          equ $60080008
HCTSIZ_DATA1_BASE       equ $40080000
HCTSIZ_PKTCNT_1          equ $00080000
HCTSIZ_PID_DATA1         equ $40000000

MBOX_TX_FULL    equ $80000000
MBOX_RX_EMPTY   equ $40000000
MBOX_PROPERTY   equ 8
TAG_SET_POWER   equ $00028001
DEV_USB_HCD     equ 3

PROP_ALLOC_SIZE equ $00000200
PROP_MSG_SIZE   equ 32
BULK_DMA_MAX    equ $00040000      ; 256 KiB Poseidon bulk bounce area (USB57; NTFS needs 128 KiB)
DMA_ALLOC_SIZE  equ $00040400
TIMEOUT         equ $00800000

        section code,code

Resident:
        dc.w    RT_MATCHWORD
        dc.l    Resident
        dc.l    EndResident
        dc.b    RTF_AUTOINIT
        dc.b    30
        dc.b    NT_DEVICE
        dc.b    0
        dc.l    DevName
        dc.l    IdString
        dc.l    InitTable

InitTable:
        dc.l    DEV_SIZE
        dc.l    FuncTable
        dc.l    0
        dc.l    DevInit

FuncTable:
        dc.l    DevOpen
        dc.l    DevClose
        dc.l    DevExpunge
        dc.l    DevReserved
        dc.l    DevBeginIO
        dc.l    DevAbortIO
        dc.l    -1

DevInit:
        move.l  d0,a1
        move.b  #NT_DEVICE,LN_TYPE(a1)
        clr.b   LN_PRI(a1)
        move.l  #DevName,LN_NAME(a1)
        clr.b   LIB_FLAGS(a1)
        move.w  #74,LIB_VERSION(a1)
        move.w  #0,LIB_REVISION(a1)
        move.l  #IdString,LIB_IDSTRING(a1)
        move.l  a6,DEV_SYSBASE(a1)
        move.l  a6,SysBasePtr
        move.w  #UHSF_OPERATIONAL,driver_state
        move.l  a1,d0
        rts

DevOpen:
        cmp.l   #0,d0
        bne.w   .badunit
        cmp.w   #UIO_SIZE,18(a1)
        bcs.w   .badlen

        ; USB57: EnsureHardware uses A6 as ExecBase internally. Preserve the
        ; real device base so io_Device and lib_OpenCnt are updated on
        ; berrypi3ap.device rather than accidentally on exec.library. Unit 0 is
        ; represented by a NULL io_Unit pointer in this single-unit HCD.
        movem.l d1-d7/a0-a6,-(sp)
        bsr     EnsureHardware
        movem.l (sp)+,d1-d7/a0-a6
        tst.l   d0
        bne.w   .hwfail

        addq.w  #1,LIB_OPENCNT(a6)
        move.l  a6,IO_DEVICE(a1)
        clr.l   IO_UNIT(a1)
        ; Development ABI export for diagnostic launcher: return the real
        ; relocated entry addresses before the launcher clears these fields.
        move.l  #DevBeginIO,UIO_DRIVERPRIVATE1(a1)
        move.l  #DevAbortIO,UIO_DRIVERPRIVATE2(a1)
        clr.b   IO_ERROR(a1)
        moveq   #0,d0
        rts

.badlen:
        move.b  #-4,IO_ERROR(a1)
        moveq   #-1,d0
        rts
.badunit:
        move.b  #-1,IO_ERROR(a1)
        moveq   #-1,d0
        rts
.hwfail:
        move.b  #ERR_HCI_ERROR,IO_ERROR(a1)
        moveq   #-1,d0
        rts

DevClose:
        tst.w   LIB_OPENCNT(a6)
        beq.s   .done
        subq.w  #1,LIB_OPENCNT(a6)
.done:
        clr.l   IO_DEVICE(a1)
        clr.l   IO_UNIT(a1)
        moveq   #0,d0
        rts

DevExpunge:
        moveq   #0,d0
        rts

DevReserved:
        moveq   #0,d0
        rts

DevBeginIO:
        ; USB57: the current DWC2 core uses one physical host channel and
        ; shared transfer state. Poseidon may call BeginIO concurrently from
        ; hub.class and massstorage.class tasks, so serialize *all* requests
        ; before publishing CurrentIOReq. This removes the race that was
        ; invisible in the single-task diagnostic launcher.
        movem.l d1-d7/a0-a6,-(sp)
        bsr     AcquireIOGate
        move.l  a1,CurrentIOReq
        clr.b   IO_ERROR(a1)
        clr.l   UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        ; Entry diagnostics: if these survive, BeginIO was reached but no DWC2 stage ran.
        move.l  #$B1400000,UIO_DRIVERPRIVATE1(a1)
        moveq   #0,d0
        move.w  IO_COMMAND(a1),d0
        swap    d0
        move.w  UIO_DIRECTION(a1),d0
        move.l  d0,UIO_DRIVERPRIVATE2(a1)

        cmp.w   #CMD_DEVICE_QUERY,IO_COMMAND(a1)
        beq.w   DoDeviceQuery
        cmp.w   #CMD_USB_RESET,IO_COMMAND(a1)
        beq.w   DoUsbReset
        cmp.w   #CMD_RESET,IO_COMMAND(a1)
        beq.w   DoUsbReset
        cmp.w   #CMD_USB_RESUME,IO_COMMAND(a1)
        beq.w   DoUsbOperational
        cmp.w   #CMD_START,IO_COMMAND(a1)
        beq.w   DoUsbOperational
        cmp.w   #CMD_STOP,IO_COMMAND(a1)
        beq.w   DoUsbSuspend
        cmp.w   #CMD_FLUSH,IO_COMMAND(a1)
        beq.w   DoFlush
        cmp.w   #CMD_REQUEST_CONTROL,IO_COMMAND(a1)
        beq.w   DoControlRequest
        cmp.w   #CMD_REQUEST_INTERRUPT,IO_COMMAND(a1)
        beq.w   DoInterruptRequest
        cmp.w   #CMD_REQUEST_BULK,IO_COMMAND(a1)
        beq.w   DoBulkRequest

        move.b  #IOERR_NOCMD,IO_ERROR(a1)
        bra.w   BeginIODone

; ------------------------------------------------
; CMD_DEVICE_QUERY
; ------------------------------------------------
DoDeviceQuery:
        move.l  UIO_DATA_BUFFER(a1),a0
        move.l  a0,d0
        tst.l   d0
        beq.w   QueryBadParam

        moveq   #0,d7
QueryNext:
        move.l  (a0),d0
        beq.w   QueryDone
        move.l  4(a0),a2
        move.l  a2,d1
        tst.l   d1
        beq.w   QuerySkip

        cmp.l   #TAG_DRIVER_STATE,d0
        beq.w   QueryState
        cmp.l   #TAG_DEVICE_VENDOR,d0
        beq.w   QueryVendor
        cmp.l   #TAG_DEVICE_PRODUCT,d0
        beq.w   QueryProduct
        cmp.l   #TAG_DEVICE_VERSION,d0
        beq.w   QueryDevVer
        cmp.l   #TAG_DEVICE_REVISION,d0
        beq.w   QueryDevRev
        cmp.l   #TAG_DRIVER_DESC,d0
        beq.w   QueryDesc
        cmp.l   #TAG_DRIVER_LICENSE,d0
        beq.w   QueryLicense
        cmp.l   #TAG_DRIVER_VERSION,d0
        beq.w   QueryDrvVer
        cmp.l   #TAG_DRIVER_FEATURES,d0
        beq.w   QueryFeatures
        bra.w   QuerySkip

QueryState:
        moveq   #0,d1
        move.w  driver_state,d1
        move.l  d1,(a2)
        move.w  d1,UIO_STATE(a1)
        addq.l  #1,d7
        bra.w   QuerySkip
QueryVendor:
        move.l  #VendorString,(a2)
        addq.l  #1,d7
        bra.w   QuerySkip
QueryProduct:
        move.l  #ProductString,(a2)
        addq.l  #1,d7
        bra.w   QuerySkip
QueryDevVer:
        move.l  #55,(a2)
        addq.l  #1,d7
        bra.w   QuerySkip
QueryDevRev:
        clr.l   (a2)
        addq.l  #1,d7
        bra.w   QuerySkip
QueryDesc:
        move.l  #DescriptionString,(a2)
        addq.l  #1,d7
        bra.w   QuerySkip
QueryLicense:
        move.l  #LicenseString,(a2)
        addq.l  #1,d7
        bra.w   QuerySkip
QueryDrvVer:
        move.l  #$00000200,(a2)
        addq.l  #1,d7
        bra.w   QuerySkip
QueryFeatures:
        move.l  #DRIVER_FEATURES,(a2)
        addq.l  #1,d7
QuerySkip:
        addq.l  #8,a0
        bra.w   QueryNext
QueryDone:
        move.l  d7,UIO_ACTUAL_LENGTH(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone
QueryBadParam:
        move.b  #ERR_BAD_PARAMETERS,IO_ERROR(a1)
        bra.w   BeginIODone

; ------------------------------------------------
; UHCMD_USBRESET / CMD_RESET
;
; Poseidon issues UHCMD_QUERYDEVICE followed by UHCMD_USBRESET before it
; starts enumerating address 0. Reset the real DWC2 root port, restore all
; endpoint data toggles to DATA0 and report a high-speed operational bus.
; ------------------------------------------------
DoUsbReset:
        ; USB57: UHCMD_USBRESET resets the *bus/root-hub state*, not the
        ; existence of the controller.  A real Poseidon HCD exposes a virtual
        ; root hub even when no physical device is attached.
        move.w  #UHSF_RESET,driver_state
        move.w  #UHSF_RESET,UIO_STATE(a1)

        bsr     root_port_power_only
        move.l  CurrentIOReq,a1
        tst.l   d0
        bne.s   .usbreset_fail

        clr.b   current_addr
        clr.b   root_hub_addr
        clr.b   root_hub_config
        clr.b   bulk_out_toggle
        clr.b   bulk_in_toggle
        clr.b   intr_in_toggle
        bsr     intr_toggle_clear_all
        clr.w   root_change_bits
        bsr     update_root_changes

        move.w  #UHSF_OPERATIONAL,driver_state
        move.w  #UHSF_OPERATIONAL,UIO_STATE(a1)
        andi.w  #$FFFC,UIO_FLAGS(a1)
        ; The virtual root hub is a USB 2.0 high-speed root hub regardless of
        ; whether its single physical port is currently occupied.
        ori.w   #UHFF_HIGHSPEED,UIO_FLAGS(a1)
        clr.l   UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        bsr     read_hprt
        move.l  CurrentIOReq,a1
        move.l  d6,UIO_DRIVERPRIVATE1(a1)
        move.l  #$52483536,UIO_DRIVERPRIVATE2(a1) ; 'RH56'
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone
.usbreset_fail:
        move.w  #UHSF_RESET,driver_state
        move.w  #UHSF_RESET,UIO_STATE(a1)
        move.b  #ERR_HCI_ERROR,IO_ERROR(a1)
        bra.w   BeginIODone

; UHCMD_USBRESUME / CMD_START: return the controller to operational state.
DoUsbOperational:
        move.w  #UHSF_OPERATIONAL,driver_state
        move.w  #UHSF_OPERATIONAL,UIO_STATE(a1)
        andi.w  #$FFFC,UIO_FLAGS(a1)
        ori.w   #UHFF_HIGHSPEED,UIO_FLAGS(a1)
        clr.l   UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

; UHCMD_USBSUSPEND / CMD_STOP.  USB57 deliberately keeps the DWC2 block
; powered for this first Trident attachment test; Poseidon still sees and can
; query the correct lifecycle state, and CMD_START/USBRESUME restores it.
DoUsbSuspend:
        move.w  #UHSF_SUSPENDED,driver_state
        move.w  #UHSF_SUSPENDED,UIO_STATE(a1)
        clr.l   UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

; All transfers are synchronous in USB57, so there is no software request
; queue to flush.  Accept CMD_FLUSH as a successful no-op instead of returning
; IOERR_NOCMD to Poseidon.
DoFlush:
        clr.l   UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

; ------------------------------------------------
; CMD_REQUEST_CONTROL
;
; USB57 supports the proven EP0 control transfers used by hub reset diagnostics:
;   - IN data stage, 1..64 bytes (proven USB25 path)
;   - zero-length host-to-device request (SETUP OUT -> STATUS IN), e.g. SET_ADDRESS
; iouh_Dir remains UHDIR_SETUP (0); bmRequestType selects the USB data direction.
; ------------------------------------------------
DoControlRequest:
        ; Preserve request pointer separately because low-level code uses a1.
        move.l  a1,CurrentIOReq

        cmp.w   #0,UIO_ENDPOINT(a1)
        bne.w   CtrlBadParam
        cmp.w   #UHDIR_SETUP,UIO_DIRECTION(a1)
        bne.w   CtrlBadParam

        moveq   #0,d0
        move.w  UIO_VIRTUAL_ADDRESS(a1),d0
        cmp.l   #127,d0
        bhi.w   CtrlBadParam
        move.b  d0,current_addr

        ; Copy setup byte fields.
        move.b  UIO_SETUP(a1),req_bm
        move.b  UIO_SETUP+1(a1),req_b

        ; Parse little-endian setup words to host integer values.
        moveq   #0,d0
        move.b  UIO_SETUP+3(a1),d0
        lsl.w   #8,d0
        move.b  UIO_SETUP+2(a1),d0
        move.w  d0,req_value

        moveq   #0,d0
        move.b  UIO_SETUP+5(a1),d0
        lsl.w   #8,d0
        move.b  UIO_SETUP+4(a1),d0
        move.w  d0,req_index

        moveq   #0,d0
        move.b  UIO_SETUP+7(a1),d0
        lsl.w   #8,d0
        move.b  UIO_SETUP+6(a1),d0
        move.w  d0,req_length

        ; USB57 root-hub emulation.  At startup root_hub_addr is zero, so
        ; Poseidon's first address-0 enumeration is answered by the virtual
        ; hub.  After SET_ADDRESS, address 0 is free for a real device on the
        ; physical DWC2 port.
        moveq   #0,d0
        move.b  current_addr,d0
        moveq   #0,d1
        move.b  root_hub_addr,d1
        cmp.b   d1,d0
        beq.w   DoRootHubControl

        ; USB60: keep the complete USB57 control engine, but use Poseidon's
        ; transport metadata for real devices.  This is deliberately minimal:
        ; no split engine, no retry rewrite, no root-hub/hotplug changes.
        moveq   #0,d0
        move.w  UIO_RESERVED1(a1),d0      ; iouh_MaxPktSize for endpoint 0
        bne.s   .ctrl60_mps_ok
        moveq   #64,d0                    ; defensive legacy fallback
.ctrl60_mps_ok:
        cmpi.w  #64,d0
        bhi.w   CtrlBadParam
        move.w  d0,ctrl_mps
        move.w  UIO_FLAGS(a1),ctrl_flags
        move.w  UIO_SPLIT_HUB_ADDR(a1),ctrl_split_hub
        move.w  UIO_SPLIT_HUB_PORT(a1),ctrl_split_port
        clr.w   ctrl_split_effective

        ; USB67: effective split is opt-in only. Raw Poseidon split metadata is
        ; never consulted again after this topology gate, so a direct FS/LS
        ; keyboard/mouse cannot accidentally fall into the split engine.
        ; Direct/root/high-speed paths remain exactly USB64.
        move.w  ctrl_flags,d0
        btst    #5,d0                     ; UHFF_SPLITTRANS
        beq.w   .ctrl65_nosplit
        btst    #1,d0                     ; UHFF_HIGHSPEED target never needs split
        bne.w   .ctrl65_nosplit

        ; A real downstream split is possible only while the physical DWC2
        ; root-port partner itself is high-speed (the external HS hub).
        bsr     read_hprt
        move.l  d6,d0
        btst    #0,d0                     ; physical root partner still connected
        beq.w   .ctrl65_nosplit
        and.l   #HPRT_SPD_MASK,d0
        bne.w   .ctrl65_nosplit           ; direct FS/LS: exact USB64 path

        ; Invalid/stale split metadata must fall back to USB64, never fail a
        ; previously working direct transfer. Ports are 1-based.
        tst.w   ctrl_split_hub
        beq.w   .ctrl65_nosplit
        tst.w   ctrl_split_port
        beq.w   .ctrl65_nosplit
        cmpi.w  #127,ctrl_split_hub
        bhi.w   .ctrl65_nosplit
        cmpi.w  #127,ctrl_split_port
        bhi.w   .ctrl65_nosplit
        move.w  #1,ctrl_split_effective

        ; Validate the same request shapes USB64 already supports: zero-length
        ; host-to-device control or device-to-host control data.
        moveq   #0,d0
        move.w  req_length,d0
        beq.s   .ctrl65_split_nodata
        moveq   #0,d1
        move.b  req_bm,d1
        btst    #7,d1
        beq.w   CtrlBadParam
        move.l  UIO_DATA_BUFFER(a1),d1
        beq.w   CtrlBadParam
        move.l  UIO_DATA_BUF_LEN(a1),d1
        cmp.l   d0,d1
        bcs.w   CtrlBadParam
        bra.s   .ctrl65_split_valid
.ctrl65_split_nodata:
        moveq   #0,d1
        move.b  req_bm,d1
        btst    #7,d1
        bne.w   CtrlBadParam
.ctrl65_split_valid:

        ; Split EP0 needs one downstream transaction per max packet. Keep the
        ; already-proven USB64 control engine untouched for every non-split
        ; request and route only split traffic through the packetised helper.
        bsr     control_split_request
        move.l  d0,d7
        move.l  CurrentIOReq,a1
        move.l  ctrl_setup_status,UIO_DRIVERPRIVATE1(a1)
        move.l  ctrl_data_status,UIO_DRIVERPRIVATE2(a1)
        move.l  d7,d0
        tst.l   d0
        bne.w   CtrlTransferFail

        moveq   #0,d0
        move.w  req_length,d0
        beq.w   CtrlNoDataSuccess

        ; Copy only bytes actually returned by the split EP0 data stage.
        moveq   #0,d1
        move.b  req_bm,d1
        btst    #7,d1
        beq.w   CtrlBadParam              ; USB65 split INPUT first: IN data only
        move.l  ctrl_actual,d2
        beq.s   .ctrl65_split_done
        move.l  UIO_DATA_BUFFER(a1),a2
        move.l  data_ptr,a0
        move.l  d2,d3
        subq.l  #1,d3
.ctrl65_copy:
        move.b  (a0)+,(a2)+
        dbra    d3,.ctrl65_copy
.ctrl65_split_done:
        move.l  ctrl_actual,UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

.ctrl65_nosplit:
        ; USB57: D0 above held the target USB address for root-hub routing.
        ; Reload the actual SETUP wLength before deciding whether this is a
        ; no-data request. USB56 accidentally tested the address as wLength,
        ; so a real device at address 0 turned GET_DESCRIPTOR(...,8) into the
        ; zero-length path and returned UHIOERR_BADPARAMS.
        moveq   #0,d0
        move.w  req_length,d0

        ; wLength==0: host-to-device request with no data phase.
        tst.w   d0
        beq.w   CtrlNoData

        ; Existing/proven IN-data path.
        cmp.w   #64,d0
        bhi.w   CtrlBadParam
        move.l  UIO_DATA_BUFFER(a1),d1
        beq.w   CtrlBadParam
        move.l  UIO_DATA_BUF_LEN(a1),d1
        cmp.l   d0,d1
        bcs.w   CtrlBadParam

        moveq   #0,d1
        move.b  req_bm,d1
        btst    #7,d1
        beq.w   CtrlBadParam

        bsr     control_in
        move.l  d0,d7

        ; private1 = SETUP HCINT, private2 = DATA HCINT.
        move.l  CurrentIOReq,a1
        move.l  ctrl_setup_status,UIO_DRIVERPRIVATE1(a1)
        move.l  ctrl_data_status,UIO_DRIVERPRIVATE2(a1)

        move.l  d7,d0
        tst.l   d0
        bne.w   CtrlTransferFail

        ; Copy returned bytes to Poseidon caller buffer.
        move.l  CurrentIOReq,a1
        move.l  UIO_DATA_BUFFER(a1),a2
        move.l  data_ptr,a0
        moveq   #0,d0
        move.w  req_length,d0
        move.l  d0,d2
        subq.l  #1,d2
CtrlCopy:
        move.b  (a0)+,(a2)+
        dbra    d2,CtrlCopy

        moveq   #0,d0
        move.w  req_length,d0
        move.l  d0,UIO_ACTUAL_LENGTH(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

CtrlNoData:
        ; A zero-length request must be host-to-device. For USB57 this covers
        ; standard SET_ADDRESS: 00 05 aa 00 00 00 00 00.
        moveq   #0,d1
        move.b  req_bm,d1
        btst    #7,d1
        bne.w   CtrlBadParam

        bsr     control_nodata_out
        move.l  d0,d7

        ; private1 = SETUP HCINT, private2 = STATUS-IN HCINT.
        move.l  CurrentIOReq,a1
        move.l  ctrl_setup_status,UIO_DRIVERPRIVATE1(a1)
        move.l  ctrl_status_status,UIO_DRIVERPRIVATE2(a1)

        move.l  d7,d0
        tst.l   d0
        bne.w   CtrlTransferFail

CtrlNoDataSuccess:
        ; A successful SET_CONFIGURATION starts all non-control endpoints at DATA0.
        cmpi.b  #$09,req_b
        bne.s   .check_clear_halt
        clr.b   bulk_out_toggle
        clr.b   bulk_in_toggle
        clr.b   intr_in_toggle
        bsr     intr_toggle_clear_all
        bra.s   .no_bulk_toggle_reset

.check_clear_halt:
        ; USB57: standard CLEAR_FEATURE(ENDPOINT_HALT) resets the data toggle
        ; of the addressed endpoint.  BOT Reset Recovery relies on this.
        cmpi.b  #$01,req_b              ; CLEAR_FEATURE
        bne.s   .no_bulk_toggle_reset
        cmpi.b  #$02,req_bm             ; standard, host-to-device, endpoint
        bne.s   .no_bulk_toggle_reset
        tst.w   req_value               ; ENDPOINT_HALT selector = 0
        bne.s   .no_bulk_toggle_reset
        moveq   #0,d0
        move.w  req_index,d0
        btst    #7,d0                   ; endpoint address direction bit
        beq.s   .clear_out_toggle
        clr.b   bulk_in_toggle
        bsr     intr_toggle_clear_all
        bra.s   .no_bulk_toggle_reset
.clear_out_toggle:
        clr.b   bulk_out_toggle
        bsr     intr_toggle_clear_all
.no_bulk_toggle_reset:
        clr.l   UIO_ACTUAL_LENGTH(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

CtrlTransferFail:
        move.l  CurrentIOReq,a1
        ; Return the low HCINT bits as extended diagnostic on failure.
        move.l  ctrl_error_hcint,d0
        move.w  d0,UIO_EXTERROR(a1)
        ; Distinguish our polling timeout marker if present.
        move.l  ctrl_error_hcint,d0
        btst    #31,d0
        beq.s   CtrlHCIError
        move.b  #ERR_TIMEOUT,IO_ERROR(a1)
        bra.w   BeginIODone
CtrlHCIError:
        move.b  #ERR_HCI_ERROR,IO_ERROR(a1)
        bra.w   BeginIODone

CtrlBadParam:
        move.l  CurrentIOReq,a1
        move.b  #ERR_BAD_PARAMETERS,IO_ERROR(a1)
        bra.w   BeginIODone

; ------------------------------------------------
; UHCMD_INTXFER -- USB57 high-speed interrupt-IN path
;
; First implementation targets the integrated hub interrupt endpoint proven
; by enumeration (address 1, endpoint 1, MPS 1, bInterval 12), but validates
; generic endpoint/address/MPS fields and uses the standard IOUsbHWReq ABI.
; Controller-owned aligned DMA is used exactly like the proven Bulk-IN path.
; ------------------------------------------------
DoInterruptRequest:
        move.l  a1,CurrentIOReq

        moveq   #0,d0
        move.w  UIO_VIRTUAL_ADDRESS(a1),d0
        cmpi.l  #127,d0
        bhi.w   IntrBadParam
        move.b  d0,current_addr
        move.w  UIO_FLAGS(a1),intr_flags
        move.w  UIO_SPLIT_HUB_ADDR(a1),intr_split_hub
        move.w  UIO_SPLIT_HUB_PORT(a1),intr_split_port
        clr.w   intr_split_effective

        ; Virtual root-hub interrupt endpoint (EP1).  No DWC2 host channel is
        ; consumed here; the bitmap is generated from HPRT0 change state.
        moveq   #0,d1
        move.b  root_hub_addr,d1
        cmp.b   d1,d0
        beq.w   DoRootHubInterrupt

        moveq   #0,d0
        move.w  UIO_ENDPOINT(a1),d0
        beq.w   IntrBadParam
        cmpi.w  #15,d0
        bhi.w   IntrBadParam
        move.w  d0,intr_endpoint

        moveq   #0,d0
        move.w  UIO_DIRECTION(a1),d0
        cmpi.w  #DIRECTION_IN,d0
        bne.w   IntrBadParam

        moveq   #0,d0
        move.w  UIO_RESERVED1(a1),d0
        beq.w   IntrBadParam
        cmpi.w  #1024,d0
        bhi.w   IntrBadParam
        move.w  d0,intr_mps

        move.l  UIO_DATA_BUF_LEN(a1),d0
        beq.w   IntrBadParam
        moveq   #0,d1
        move.w  intr_mps,d1
        cmp.l   d1,d0
        bhi.w   IntrBadParam             ; one interrupt transaction for USB57
        move.l  d0,intr_length
        move.l  UIO_DATA_BUFFER(a1),a0
        move.l  a0,d0
        beq.w   IntrBadParam
        move.l  a0,intr_buffer

        moveq   #0,d0
        move.w  UIO_RESERVED2(a1),d0
        bne.s   .intr_interval_ok
        moveq   #1,d0
.intr_interval_ok:
        move.w  d0,intr_interval

        move.w  intr_flags,d0
        btst    #5,d0                     ; UHFF_SPLITTRANS
        beq.s   .intr65_transport_ok
        btst    #1,d0                     ; HS target: direct transaction
        bne.s   .intr65_transport_ok

        ; Periodic split-IN is legal only when the physical root-port partner
        ; is a connected high-speed hub. Otherwise preserve USB64 exactly.
        bsr     read_hprt
        move.l  d6,d0
        btst    #0,d0
        beq.s   .intr65_transport_ok
        and.l   #HPRT_SPD_MASK,d0
        bne.s   .intr65_transport_ok

        tst.w   intr_split_hub
        beq.s   .intr65_transport_ok
        tst.w   intr_split_port
        beq.s   .intr65_transport_ok
        cmpi.w  #127,intr_split_hub
        bhi.s   .intr65_transport_ok
        cmpi.w  #127,intr_split_port
        bhi.s   .intr65_transport_ok
        move.w  #1,intr_split_effective
.intr65_transport_ok:

        ; Reserve one full max packet in the aligned DWC2 DMA workspace.
        move.l  data_ptr,a0
        move.l  a0,intr_dma_buffer
        moveq   #0,d0
        move.w  intr_mps,d0
        move.l  d0,intr_dma_length
        move.l  a0,a2
        move.l  d0,d3
        subq.l  #1,d3
        move.b  #$CC,d4
.intr_poison:
        move.b  d4,(a2)+
        dbra    d3,.intr_poison

        moveq   #0,d1                    ; DMA writes RAM
        move.l  d1,intr_dma_flags
        move.l  intr_dma_buffer,a0
        move.l  intr_dma_length,d0
        bsr     dma_pre
        tst.l   d0
        bne.w   IntrDMABad

        ; One-packet interrupt IN. USB61 keeps DATA toggle per device/address
        ; and endpoint, because composite HID devices may expose more than one
        ; interrupt-IN pipe.
        move.l  intr_dma_length,d4
        or.l    #HCTSIZ_PKTCNT_1,d4
        bsr     intr_toggle_get
        tst.b   d0
        beq.s   .intr_pid_data0
        or.l    #HCTSIZ_PID_DATA1,d4
.intr_pid_data0:
        move.l  d4,d0
.intr_pid_ready:
        move.l  d0,intr_hctsiz
        clr.l   intr_final_hctsiz
        clr.l   intr_actual

        move.l  CurrentIOReq,a1
        move.l  UIO_TIMEOUT(a1),d0
        bne.s   .intr_retry_budget_ok
        move.l  #2000,d0
.intr_retry_budget_ok:
        move.l  d0,intr_retry_left
        clr.l   intr_retry_count

.intr_try:
        tst.w   intr_split_effective
        beq.s   .intr65_direct
        move.l  intr_hctsiz,d0
        move.l  dma_bus,d2
        bsr     channel0_run_split_interrupt
        move.l  d0,intr_hcint
        bra.s   .intr65_after_run
.intr65_direct:
        move.l  intr_hctsiz,d0
        move.l  dma_bus,d2
        bsr     channel0_start_interrupt
        tst.l   d0
        bne.w   IntrStartBad
        bsr     channel0_wait
        move.l  d0,intr_hcint
.intr65_after_run:
        move.l  #HCTSIZ0,a0
        bsr     mmio_read32
        move.l  d0,intr_final_hctsiz

        move.l  intr_hcint,d0
        bsr     transfer_status_ok
        tst.l   d0
        beq.w   .intr_xfer_ok

        ; USB57 Trident stability: hub.class treats UHIOERR_TIMEOUT specially
        ; as proof that the hub itself vanished. A rare local DWC2 channel-wait
        ; timeout must therefore never be propagated for the integrated hub
        ; while the physical root port is still connected and enabled. First
        ; halt/clean channel 0, then turn this into the same empty status poll
        ; used for an ordinary idle NAK. If the root port is really gone, keep
        ; the genuine timeout so Poseidon can tear the hub down correctly.
        move.l  intr_hcint,d0
        btst    #31,d0
        beq.s   .intr_after_local_timeout
        cmpi.b  #1,current_addr
        bne.w   .intr_no_retry
        cmpi.w  #1,intr_endpoint
        bne.w   .intr_no_retry
        cmpi.w  #1,intr_mps
        bne.w   .intr_no_retry
        cmpi.l  #1,intr_length
        bne.w   .intr_no_retry
        bsr     channel0_recover_timeout
        bsr     read_hprt
        move.l  d6,d0
        and.l   #HPRT_CONN+HPRT_ENA,d0
        cmpi.l  #HPRT_CONN+HPRT_ENA,d0
        bne.w   .intr_no_retry
        addq.l  #1,intr_recovered_timeouts
        bra.w   .intr_hub_idle_nak
.intr_after_local_timeout:

        ; Periodic IN endpoints normally NAK while no report/change is ready.
        ; A NAK is therefore not a failed hub status packet. The classic
        ; Poseidon ABI says iouh_NakTimeout only retires a request when
        ; UHFF_NAKTIMEOUT is set. USB57 special-cases the Pi's integrated hub
        ; status pipe (addr 1 / EP1 / MPS1 / 1 byte): a NAK is converted into
        ; a synthetic zero change bitmap after a short cooperative delay.
        ; This keeps the synchronous development driver usable under Trident
        ; without monopolising channel 0 indefinitely. DATA PID is unchanged.
        move.l  intr_hcint,d0
        btst    #31,d0
        bne.w   .intr_no_retry
        btst    #3,d0                     ; STALL
        bne.w   .intr_no_retry
        move.l  intr_hcint,d0
        and.l   #HCINT_ERROR_MASK,d0
        bne.w   .intr_no_retry
        move.l  intr_hcint,d0
        btst    #4,d0                     ; NAK
        beq.w   .intr_no_retry

        ; Is this exactly the integrated hub change endpoint?
        cmpi.b  #1,current_addr
        bne.s   .intr_timed_nak
        cmpi.w  #1,intr_endpoint
        bne.s   .intr_timed_nak
        cmpi.w  #1,intr_mps
        bne.s   .intr_timed_nak
        cmpi.l  #1,intr_length
        beq.w   .intr_hub_idle_nak

.intr_timed_nak:
        ; For ordinary interrupt endpoints keep USB57's bounded retry only if
        ; the caller explicitly enabled NAK timeout semantics.
        move.l  CurrentIOReq,a1
        btst    #3,UIO_FLAGS(a1)          ; UHFF_NAKTIMEOUT
        beq.w   .intr_hid_idle_nak
        tst.l   intr_retry_left
        beq.w   .intr_nak_timeout
        subq.l  #1,intr_retry_left
        addq.l  #1,intr_retry_count
        bsr     bulk_wait_next_frame
        bra.w   .intr_try

.intr_nak_timeout:
        move.l  intr_dma_buffer,a0
        move.l  intr_dma_length,d0
        move.l  intr_dma_flags,d1
        bsr     dma_post
        move.l  CurrentIOReq,a1
        move.l  intr_hcint,UIO_DRIVERPRIVATE1(a1)
        move.l  intr_hctsiz,UIO_DRIVERPRIVATE2(a1)
        clr.l   UIO_ACTUAL_LENGTH(a1)
        move.w  intr_hcint,UIO_EXTERROR(a1)
        move.b  #ERR_NAK_TIMEOUT,IO_ERROR(a1)
        bra.w   BeginIODone

.intr_hub_idle_nak:
        ; DMA transaction ended with NAK, so no payload was written. Finish the
        ; cache ownership cycle, return one zero bitmap byte to hub.class and
        ; release the global channel gate *before* sleeping. This approximates
        ; a pending periodic interrupt request without blocking mass-storage
        ; traffic on our still-synchronous one-channel implementation.
        move.l  intr_dma_buffer,a0
        move.l  intr_dma_length,d0
        move.l  intr_dma_flags,d1
        bsr     dma_post

        move.l  CurrentIOReq,a5
        move.l  intr_hcint,UIO_DRIVERPRIVATE1(a5)
        move.l  intr_hctsiz,UIO_DRIVERPRIVATE2(a5)
        move.l  intr_buffer,a2
        clr.b   (a2)
        move.l  #1,UIO_ACTUAL_LENGTH(a5)
        clr.w   UIO_EXTERROR(a5)
        clr.b   IO_ERROR(a5)

        ; Do not hold the shared DWC2 gate during the cooperative idle delay.
        bsr     ReleaseIOGate
        move.l  DosBasePtr,a6
        moveq   #HUB_IDLE_DELAY_TICKS,d1
        jsr     Delay(a6)

        ; Complete this particular request from the saved A5 pointer; another
        ; BeginIO may have legitimately replaced CurrentIOReq while we slept.
        move.l  a5,a1
        btst    #0,IO_FLAGS(a1)
        bne.s   .hub_idle_quick_done
        move.l  SysBasePtr,a6
        jsr     ReplyMsg(a6)
.hub_idle_quick_done:
        movem.l (sp)+,d1-d7/a0-a6
        rts

.intr_hid_idle_nak:
        ; USB64/66: HID NAK semantics are selected per address/endpoint.
        ;
        ; A pipe which has never delivered a real report yet stays nonblocking.
        ; This is important for composite keyboards: an always-idle multimedia
        ; interface must not monopolise the one synchronous DWC2 host channel.
        ;
        ; Once a pipe has delivered at least one real report, it is considered
        ; active.  A NAK on that active pipe means "the previous HID state is
        ; still current". Keep the interrupt request pending until a genuinely
        ; new report arrives. This allows Amiga input autorepeat to see a held
        ; keyboard key, and prevents relative mouse motion from being replayed.
        bsr     intr_seen_get
        tst.l   d0
        bne.w   .intr_hid_active_nak

        ; Never-active pipe: retain USB63's cooperative nonblocking poll.  No
        ; key/button state exists yet on this pipe, so a neutral buffer is safe
        ; and keeps secondary HID interfaces from blocking useful interfaces.
        move.l  intr_dma_buffer,a0
        move.l  intr_dma_length,d0
        move.l  intr_dma_flags,d1
        bsr     dma_post

        move.l  CurrentIOReq,a5
        move.l  intr_hcint,UIO_DRIVERPRIVATE1(a5)
        move.l  intr_hctsiz,UIO_DRIVERPRIVATE2(a5)

        move.l  intr_buffer,a2
        move.l  intr_length,d3
        beq.s   .hid_idle_buf_clear_done
        subq.l  #1,d3
.hid_idle_buf_clear:
        clr.b   (a2)+
        dbra    d3,.hid_idle_buf_clear
.hid_idle_buf_clear_done:
        clr.l   UIO_ACTUAL_LENGTH(a5)
        clr.w   UIO_EXTERROR(a5)
        clr.b   IO_ERROR(a5)

        bsr     ReleaseIOGate
        move.l  DosBasePtr,a6
        moveq   #HID_IDLE_DELAY_TICKS,d1
        jsr     Delay(a6)

        move.l  a5,a1
        btst    #0,IO_FLAGS(a1)
        bne.s   .hid_idle_quick_done
        move.l  SysBasePtr,a6
        jsr     ReplyMsg(a6)
.hid_idle_quick_done:
        movem.l (sp)+,d1-d7/a0-a6
        rts

.intr_hid_active_nak:
        ; Active HID pipe: do not manufacture a zero/neutral report and do not
        ; return stale data on a NAK.
        ;
        ; USB74 fix: this used to fall through into an *unbounded* retry loop
        ; (bra.w .intr_try) that never released the single shared DWC2 gate
        ; (io_gate_busy). With only one device attached this merely delayed
        ; things; the moment a second device sat behind a hub, its BeginIO
        ; spun forever inside AcquireIOGate because this loop never gave the
        ; gate back - which is exactly the "read errors / input dead" failure
        ; with two devices on a hub.
        ;
        ; Fix: complete this poll immediately with UIO_ACTUAL_LENGTH=0 (an
        ; explicit "no new report yet", never a fabricated report) and
        ; release the gate exactly like the never-active idle-NAK path above
        ; already does correctly. hub.class resubmits the interrupt request
        ; on its own schedule, so held-key autorepeat keeps working and every
        ; pipe now gets a fair turn at the one physical channel.
        move.l  CurrentIOReq,a5
        cmpi.b  #IOERR_ABORTED,IO_ERROR(a5)
        beq.w   .intr_hid_aborted

        move.l  intr_dma_buffer,a0
        move.l  intr_dma_length,d0
        move.l  intr_dma_flags,d1
        bsr     dma_post

        move.l  CurrentIOReq,a5
        move.l  intr_hcint,UIO_DRIVERPRIVATE1(a5)
        move.l  intr_hctsiz,UIO_DRIVERPRIVATE2(a5)
        clr.l   UIO_ACTUAL_LENGTH(a5)
        clr.w   UIO_EXTERROR(a5)
        clr.b   IO_ERROR(a5)

        bsr     ReleaseIOGate
        move.l  DosBasePtr,a6
        moveq   #HID_IDLE_DELAY_TICKS,d1
        jsr     Delay(a6)

        move.l  a5,a1
        btst    #0,IO_FLAGS(a1)
        bne.s   .hid_active_quick_done
        move.l  SysBasePtr,a6
        jsr     ReplyMsg(a6)
.hid_active_quick_done:
        movem.l (sp)+,d1-d7/a0-a6
        rts

.intr_hid_aborted:
        move.l  intr_dma_buffer,a0
        move.l  intr_dma_length,d0
        move.l  intr_dma_flags,d1
        bsr     dma_post
        move.l  CurrentIOReq,a1
        clr.l   UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        ; AbortIO already stored IOERR_ABORTED in io_Error.
        bra.w   BeginIODone

.intr_no_retry:
        ; USB63: if the directly attached HID device vanished while a key was
        ; down, first deliver one neutral report.  Otherwise Amiga input may
        ; retain the last key state and start repeating it after the keyboard
        ; has already been unplugged. The root-hub poll will report the actual
        ; disconnect immediately afterwards.
        bsr     read_hprt
        move.l  d6,d0
        and.l   #HPRT_CONN,d0
        beq.s   .intr_disconnect_neutral

        move.l  intr_dma_buffer,a0
        move.l  intr_dma_length,d0
        move.l  intr_dma_flags,d1
        bsr     dma_post
        move.l  CurrentIOReq,a1
        move.l  intr_hcint,UIO_DRIVERPRIVATE1(a1)
        move.l  intr_hctsiz,UIO_DRIVERPRIVATE2(a1)
        bra.w   IntrTransferBad

.intr_disconnect_neutral:
        move.l  intr_dma_buffer,a0
        move.l  intr_dma_length,d0
        move.l  intr_dma_flags,d1
        bsr     dma_post
        move.l  CurrentIOReq,a1
        move.l  intr_hcint,UIO_DRIVERPRIVATE1(a1)
        move.l  intr_hctsiz,UIO_DRIVERPRIVATE2(a1)
        move.l  intr_buffer,a2
        move.l  intr_length,d3
        beq.s   .intr_disconnect_buf_done
        subq.l  #1,d3
.intr_disconnect_buf_clear:
        clr.b   (a2)+
        dbra    d3,.intr_disconnect_buf_clear
.intr_disconnect_buf_done:
        move.l  intr_length,UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

.intr_xfer_ok:
        move.l  intr_dma_buffer,a0
        move.l  intr_dma_length,d0
        move.l  intr_dma_flags,d1
        bsr     dma_post

        move.l  CurrentIOReq,a1
        move.l  intr_hcint,UIO_DRIVERPRIVATE1(a1)
        move.l  intr_hctsiz,UIO_DRIVERPRIVATE2(a1)

        ; DWC2 leaves the next PID encoded in final HCTSIZ. Keep it per
        ; address/endpoint so multiple HID interfaces cannot corrupt each
        ; other's DATA0/DATA1 state.
        move.l  intr_final_hctsiz,d0
        and.l   #$60000000,d0
        cmpi.l  #HCTSIZ_PID_DATA1,d0
        bne.s   .intr_next_data0
        moveq   #1,d0
        bsr     intr_toggle_set
        bra.s   .intr_pid_done
.intr_next_data0:
        moveq   #0,d0
        bsr     intr_toggle_set
.intr_pid_done:

        ; Actual = one-MPS reservation - residual XferSize.
        move.l  intr_final_hctsiz,d0
        and.l   #$0007FFFF,d0
        move.l  intr_dma_length,d2
        cmp.l   d2,d0
        bhi.s   .intr_length_bad
        sub.l   d0,d2
        move.l  d2,intr_actual
        cmp.l   intr_length,d2
        bhi.s   .intr_length_bad

        tst.l   d2
        beq.s   .intr_actual_ready
        bsr     intr_seen_set
        move.l  data_ptr,a0
        move.l  intr_buffer,a2
        move.l  d2,d3
        subq.l  #1,d3
.intr_copy:
        move.b  (a0)+,(a2)+
        dbra    d3,.intr_copy
.intr_actual_ready:
        move.l  intr_actual,UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

.intr_length_bad:
        move.w  #$7FFC,UIO_EXTERROR(a1)
        clr.l   UIO_ACTUAL_LENGTH(a1)
        move.b  #ERR_HCI_ERROR,IO_ERROR(a1)
        bra.w   BeginIODone

IntrStartBad:
        move.l  intr_dma_buffer,a0
        move.l  intr_dma_length,d0
        move.l  intr_dma_flags,d1
        bsr     dma_post
IntrDMABad:
        move.l  CurrentIOReq,a1
        move.b  #ERR_HCI_ERROR,IO_ERROR(a1)
        bra.w   BeginIODone

IntrTransferBad:
        move.l  CurrentIOReq,a1
        move.l  intr_hcint,d0
        move.w  d0,UIO_EXTERROR(a1)
        btst    #31,d0
        bne.s   .intr_timeout
        btst    #3,d0
        bne.s   .intr_stall
        btst    #4,d0
        bne.s   .intr_nak
        move.b  #ERR_HCI_ERROR,IO_ERROR(a1)
        bra.w   BeginIODone
.intr_timeout:
        move.b  #ERR_TIMEOUT,IO_ERROR(a1)
        bra.w   BeginIODone
.intr_stall:
        move.b  #4,IO_ERROR(a1)
        bra.w   BeginIODone
.intr_nak:
        move.b  #2,IO_ERROR(a1)
        bra.w   BeginIODone

IntrBadParam:
        move.l  CurrentIOReq,a1
        move.b  #ERR_BAD_PARAMETERS,IO_ERROR(a1)
        bra.w   BeginIODone

; USB61 interrupt DATA-toggle helpers.  128 device addresses * 16 endpoint
; numbers = 2048 pipes, represented by a 256-byte bitset.
; This is deliberately independent from bulk_in_toggle.
intr_toggle_get:
        movem.l d1-d2/a0,-(sp)
        moveq   #0,d1
        move.b  current_addr,d1
        lsl.w   #4,d1
        moveq   #0,d2
        move.w  intr_endpoint,d2
        andi.w  #$000F,d2
        or.w    d2,d1
        move.w  d1,d2
        lsr.w   #3,d1
        andi.w  #7,d2
        lea     intr_toggle_bits,a0
        adda.w  d1,a0
        moveq   #0,d0
        btst    d2,(a0)
        beq.s   .itg_done
        moveq   #1,d0
.itg_done:
        movem.l (sp)+,d1-d2/a0
        rts

; d0 = 0 -> DATA0 next, nonzero -> DATA1 next.
intr_toggle_set:
        movem.l d1-d3/a0,-(sp)
        move.l  d0,d3
        moveq   #0,d1
        move.b  current_addr,d1
        lsl.w   #4,d1
        moveq   #0,d2
        move.w  intr_endpoint,d2
        andi.w  #$000F,d2
        or.w    d2,d1
        move.w  d1,d2
        lsr.w   #3,d1
        andi.w  #7,d2
        lea     intr_toggle_bits,a0
        adda.w  d1,a0
        tst.l   d3
        beq.s   .its_clear
        bset    d2,(a0)
        bra.s   .its_done
.its_clear:
        bclr    d2,(a0)
.its_done:
        movem.l (sp)+,d1-d3/a0
        rts

intr_seen_get:
        movem.l d1-d2/a0,-(sp)
        moveq   #0,d1
        move.b  current_addr,d1
        lsl.w   #4,d1
        moveq   #0,d2
        move.w  intr_endpoint,d2
        andi.w  #$000F,d2
        or.w    d2,d1
        move.w  d1,d2
        lsr.w   #3,d1
        andi.w  #7,d2
        lea     intr_seen_bits,a0
        adda.w  d1,a0
        moveq   #0,d0
        btst    d2,(a0)
        beq.s   .isg_done
        moveq   #1,d0
.isg_done:
        movem.l (sp)+,d1-d2/a0
        rts

intr_seen_set:
        movem.l d1-d2/a0,-(sp)
        moveq   #0,d1
        move.b  current_addr,d1
        lsl.w   #4,d1
        moveq   #0,d2
        move.w  intr_endpoint,d2
        andi.w  #$000F,d2
        or.w    d2,d1
        move.w  d1,d2
        lsr.w   #3,d1
        andi.w  #7,d2
        lea     intr_seen_bits,a0
        adda.w  d1,a0
        bset    d2,(a0)
        movem.l (sp)+,d1-d2/a0
        rts

intr_toggle_clear_all:
        movem.l d0-d1/a0,-(sp)
        lea     intr_toggle_bits,a0
        moveq   #0,d0
        moveq   #63,d1
.itc_loop:
        move.l  d0,(a0)+
        dbra    d1,.itc_loop
        ; USB64/66: configuration/reset starts with no HID pipe classified active.
        lea     intr_seen_bits,a0
        moveq   #63,d1
.isc_loop:
        move.l  d0,(a0)+
        dbra    d1,.isc_loop
        movem.l (sp)+,d0-d1/a0
        rts

; ------------------------------------------------
; UHCMD_BULKXFER -- USB57 high-speed bulk path with NAK/NYET retry
;
; Development scope for USB57:
;   * multi-packet requests up to 256 KiB
;   * high-speed endpoints, no split transactions
;   * endpoint number in iouh_Endpoint, direction in iouh_Dir
;   * DATA0/DATA1 toggle maintained independently for IN and OUT
; USB57 uses controller-owned aligned bounce DMA for both IN and OUT, and retries NAK/NYET without advancing DATA toggle.
; ------------------------------------------------
DoBulkRequest:
        move.l  a1,CurrentIOReq

        moveq   #0,d0
        move.w  UIO_VIRTUAL_ADDRESS(a1),d0
        cmpi.l  #127,d0
        bhi.w   BulkBadParam
        move.b  d0,current_addr

        moveq   #0,d0
        move.w  UIO_ENDPOINT(a1),d0
        beq.w   BulkBadParam
        cmpi.w  #15,d0
        bhi.w   BulkBadParam
        move.w  d0,bulk_endpoint

        moveq   #0,d0
        move.w  UIO_DIRECTION(a1),d0
        cmpi.w  #DIRECTION_OUT,d0
        beq.s   .dir_ok
        cmpi.w  #DIRECTION_IN,d0
        bne.w   BulkBadParam
.dir_ok:
        move.w  d0,bulk_direction

        moveq   #0,d0
        move.w  UIO_RESERVED1(a1),d0
        beq.w   BulkBadParam
        cmpi.w  #512,d0
        bhi.w   BulkBadParam
        move.w  d0,bulk_mps

        move.l  UIO_DATA_BUF_LEN(a1),d0
        beq.w   BulkBadParam
        cmpi.l  #BULK_DMA_MAX,d0
        bhi.w   BulkBadParam
        move.l  d0,bulk_length
        move.l  UIO_DATA_BUFFER(a1),a0
        move.l  a0,d0
        beq.w   BulkBadParam
        move.l  a0,bulk_buffer

        ; USB57 multi-packet DWC2 rule:
        ; PktCnt = ceil(requested_length / MPS).  For IN, XferSize reserves
        ; PktCnt*MPS bytes because host DMA must allow the device to terminate
        ; each packet naturally; Actual is then derived from residual XferSize.
        ; The aligned controller-owned bounce area is 256 KiB in USB57; the
        ; observed NTFS3G path issues 128 KiB / 256-packet READ(10) requests.
        move.l  bulk_length,d3
        moveq   #0,d4
        move.w  bulk_mps,d4
        move.l  d3,d5
        add.l   d4,d5
        subq.l  #1,d5
        divu.w  d4,d5
        moveq   #0,d6
        move.w  d5,d6
        move.l  d6,bulk_pktcnt

        cmpi.w  #DIRECTION_IN,bulk_direction
        bne.s   .dma_out

        move.l  data_ptr,a0
        move.l  a0,bulk_dma_buffer
        move.l  d6,d0
        mulu.w  d4,d0
        move.l  d0,bulk_dma_length

        ; Poison the complete receive reservation so short-packet behavior
        ; is visible and no stale previous data is mistaken for RX.
        move.l  a0,a2
        ; USB57: 32-bit fill. USB54 used DBRA on the low 16 bits, which
        ; wraps at 64 KiB and cannot safely prepare NTFS-sized transfers.
        move.l  bulk_dma_length,d7
        move.l  d7,d5
        lsr.l   #2,d7
        beq.s   .bulk_fill_bytes
        move.l  #$CCCCCCCC,d0
.bulk_fill_longs:
        move.l  d0,(a2)+
        subq.l  #1,d7
        bne.s   .bulk_fill_longs
.bulk_fill_bytes:
        andi.l  #3,d5
        beq.s   .bulk_fill_done
        move.b  #$CC,d0
.bulk_fill_tail:
        move.b  d0,(a2)+
        subq.l  #1,d5
        bne.s   .bulk_fill_tail
.bulk_fill_done:

        moveq   #0,d1                 ; DMA writes RAM
        bra.s   .dma_selected

.dma_out:
        ; USB57: DWC2 host DMA requires DWORD-aligned transfer buffers.
        ; Poseidon callers are allowed to supply ordinary Amiga buffers, so
        ; copy Bulk-OUT payloads into our controller-owned aligned workspace
        ; instead of programming HCDMA with the caller pointer directly.
        move.l  data_ptr,a0
        move.l  a0,bulk_dma_buffer
        move.l  bulk_length,d0
        move.l  d0,bulk_dma_length
        move.l  bulk_buffer,a2
        ; USB57: 32-bit copy count; supports transfers above 64 KiB.
        move.l  bulk_length,d3
        move.l  d3,d5
        lsr.l   #2,d3
        beq.s   .copy_bulk_out_bytes
.copy_bulk_out_longs:
        move.l  (a2)+,(a0)+
        subq.l  #1,d3
        bne.s   .copy_bulk_out_longs
.copy_bulk_out_bytes:
        andi.l  #3,d5
        beq.s   .copy_bulk_out_done
.copy_bulk_out_tail:
        move.b  (a2)+,(a0)+
        subq.l  #1,d5
        bne.s   .copy_bulk_out_tail
.copy_bulk_out_done:
        move.l  #DMA_ReadFromRAM,d1   ; controller reads aligned RAM copy

.dma_selected:
        move.l  d1,bulk_dma_flags
        move.l  bulk_dma_buffer,a0
        move.l  bulk_dma_length,d0
        bsr     dma_pre
        tst.l   d0
        bne.w   BulkDMABad

        ; Program a complete multi-packet request. OUT XferSize is the exact
        ; byte count; IN XferSize is the rounded DMA reservation. PktCnt is
        ; encoded in HCTSIZ bits 28..19.
        cmpi.w  #DIRECTION_IN,bulk_direction
        bne.s   .hctsiz_out
        move.l  bulk_dma_length,d0
        bra.s   .hctsiz_len_ready
.hctsiz_out:
        move.l  bulk_length,d0
.hctsiz_len_ready:
        move.l  bulk_pktcnt,d6
        lsl.l   #8,d6
        lsl.l   #8,d6
        lsl.l   #3,d6
        or.l    d6,d0

        ; Initial PID comes from the persistent endpoint toggle.
        cmpi.w  #DIRECTION_IN,bulk_direction
        beq.s   .get_in_toggle
        tst.b   bulk_out_toggle
        beq.s   .pid_ready
        or.l    #HCTSIZ_PID_DATA1,d0
        bra.s   .pid_ready
.get_in_toggle:
        tst.b   bulk_in_toggle
        beq.s   .pid_ready
        or.l    #HCTSIZ_PID_DATA1,d0
.pid_ready:
        move.l  d0,bulk_hctsiz
        clr.l   bulk_final_hctsiz
        clr.l   bulk_actual

        ; Use Poseidon's NAK timeout as a retry budget.  NAK/NYET retries
        ; re-enable the halted channel without advancing the current DATA PID.
        move.l  CurrentIOReq,a1
        move.l  UIO_TIMEOUT(a1),d0
        bne.s   .retry_budget_ok
        move.l  #2000,d0
.retry_budget_ok:
        move.l  d0,bulk_retry_left
        clr.l   bulk_retry_count

.bulk_try:
        move.l  bulk_hctsiz,d0
        move.l  dma_bus,d2
        bsr     channel0_start_bulk
        tst.l   d0
        bne.w   BulkStartBad

        bsr     channel0_wait
        move.l  d0,bulk_hcint

        ; Capture residual transfer size/PID immediately at channel halt.
        move.l  #HCTSIZ0,a0
        bsr     mmio_read32
        move.l  d0,bulk_final_hctsiz

        ; A complete packet wins immediately.
        move.l  bulk_hcint,d0
        bsr     transfer_status_ok
        tst.l   d0
        beq.s   .bulk_xfer_ok

        ; Never retry timeout, STALL or a hard transaction error.
        ; Retry only halted NAK/NYET, preserving DMA mapping and DATA PID.
        move.l  bulk_hcint,d0
        btst    #31,d0
        bne.s   .bulk_no_retry
        btst    #3,d0                  ; STALL
        bne.s   .bulk_no_retry
        move.l  bulk_hcint,d0
        and.l   #HCINT_ERROR_MASK,d0
        bne.s   .bulk_no_retry
        move.l  bulk_hcint,d0
        btst    #4,d0                  ; NAK
        bne.s   .bulk_retry
        btst    #6,d0                  ; NYET
        beq.s   .bulk_no_retry
.bulk_retry:
        tst.l   bulk_retry_left
        beq.s   .bulk_no_retry
        subq.l  #1,bulk_retry_left
        addq.l  #1,bulk_retry_count
        bsr     bulk_wait_next_frame
        bra.w   .bulk_try

.bulk_no_retry:
        move.l  bulk_dma_buffer,a0
        move.l  bulk_dma_length,d0
        move.l  bulk_dma_flags,d1
        bsr     dma_post
        move.l  CurrentIOReq,a1
        move.l  bulk_hcint,UIO_DRIVERPRIVATE1(a1)
        move.l  bulk_hctsiz,UIO_DRIVERPRIVATE2(a1)
        bra.w   BulkTransferBad

.bulk_xfer_ok:
        ; Complete cache/DMA handoff before CPU reads the bounce buffer.
        move.l  bulk_dma_buffer,a0
        move.l  bulk_dma_length,d0
        move.l  bulk_dma_flags,d1
        bsr     dma_post

        move.l  CurrentIOReq,a1
        move.l  bulk_hcint,UIO_DRIVERPRIVATE1(a1)
        move.l  bulk_hctsiz,UIO_DRIVERPRIVATE2(a1)

        ; Derive the next DATA PID from DWC2's final HCTSIZ PID field instead
        ; of blindly flipping it in software.
        move.l  bulk_final_hctsiz,d0
        and.l   #$60000000,d0
        cmpi.l  #HCTSIZ_PID_DATA1,d0
        bne.s   .next_pid_data0
        cmpi.w  #DIRECTION_IN,bulk_direction
        bne.s   .set_out_data1
        move.b  #1,bulk_in_toggle
        bra.s   .pid_after_done
.set_out_data1:
        move.b  #1,bulk_out_toggle
        bra.s   .pid_after_done
.next_pid_data0:
        cmpi.w  #DIRECTION_IN,bulk_direction
        bne.s   .set_out_data0
        clr.b   bulk_in_toggle
        bra.s   .pid_after_done
.set_out_data0:
        clr.b   bulk_out_toggle
.pid_after_done:

        ; OUT success means all requested bytes were sent.
        cmpi.w  #DIRECTION_IN,bulk_direction
        beq.s   .calc_in_actual
        move.l  bulk_length,d2
        move.l  d2,bulk_actual
        bra.s   .actual_ready

.calc_in_actual:
        ; For IN, actual = programmed rounded reservation - residual XferSize.
        move.l  bulk_final_hctsiz,d0
        and.l   #$0007FFFF,d0
        move.l  bulk_dma_length,d2
        cmp.l   d2,d0
        bhi.s   .bulk_in_length_bad
        sub.l   d0,d2
        move.l  d2,bulk_actual

        ; The device must not return more data than Poseidon requested.
        cmp.l   bulk_length,d2
        bhi.s   .bulk_in_length_bad

        ; Copy only bytes actually received from the aligned bounce buffer.
        tst.l   d2
        beq.s   .actual_ready
        move.l  data_ptr,a0
        move.l  bulk_buffer,a2
        ; USB57: 32-bit copy count; NTFS may request 128 KiB at once.
        move.l  d2,d3
        move.l  d3,d5
        lsr.l   #2,d3
        beq.s   .copy_bulk_in_bytes
.copy_bulk_in_longs:
        move.l  (a0)+,(a2)+
        subq.l  #1,d3
        bne.s   .copy_bulk_in_longs
.copy_bulk_in_bytes:
        andi.l  #3,d5
        beq.s   .actual_ready
.copy_bulk_in_tail:
        move.b  (a0)+,(a2)+
        subq.l  #1,d5
        bne.s   .copy_bulk_in_tail

.actual_ready:
        move.l  bulk_actual,UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

.bulk_in_length_bad:
        ; Synthetic marker: DWC2 completed but residual size is inconsistent
        ; with the one-packet reservation / caller length.
        move.w  #$7FFD,UIO_EXTERROR(a1)
        clr.l   UIO_ACTUAL_LENGTH(a1)
        move.b  #ERR_HCI_ERROR,IO_ERROR(a1)
        bra.w   BeginIODone

BulkStartBad:
        move.l  bulk_dma_buffer,a0
        move.l  bulk_dma_length,d0
        move.l  bulk_dma_flags,d1
        bsr     dma_post
BulkDMABad:
        move.l  CurrentIOReq,a1
        move.b  #ERR_HCI_ERROR,IO_ERROR(a1)
        bra.w   BeginIODone

BulkTransferBad:
        move.l  CurrentIOReq,a1
        move.l  bulk_hcint,d0
        move.w  d0,UIO_EXTERROR(a1)
        btst    #31,d0
        bne.s   .bulk_timeout
        btst    #3,d0
        bne.s   .bulk_stall
        move.b  #ERR_HCI_ERROR,IO_ERROR(a1)
        bra.w   BeginIODone
.bulk_timeout:
        move.b  #ERR_TIMEOUT,IO_ERROR(a1)
        bra.w   BeginIODone
.bulk_stall:
        move.b  #4,IO_ERROR(a1)
        bra.w   BeginIODone

BulkBadParam:
        move.l  CurrentIOReq,a1
        move.b  #ERR_BAD_PARAMETERS,IO_ERROR(a1)

BeginIODone:
        ; USB57: synchronous HCD operations may still arrive through SendIO().
        ; Keep the gate held until ReplyMsg has consumed the current request;
        ; otherwise a woken Poseidon task could overwrite CurrentIOReq before
        ; this completion path is finished.
        move.l  CurrentIOReq,a1
        btst    #0,IO_FLAGS(a1)         ; IOF_QUICK
        bne.s   .beginio_quick_done
        move.l  SysBasePtr,a6
        jsr     ReplyMsg(a6)
.beginio_quick_done:
        bsr     ReleaseIOGate
        movem.l (sp)+,d1-d7/a0-a6
        rts

; ------------------------------------------------
; USB57 single-channel BeginIO gate
;
; Poseidon can have hub.class and massstorage.class issue requests from
; different tasks. The current development core has one DWC2 host channel and
; shared global transfer state, so concurrent BeginIO execution is unsafe.
; QUICKIO is not advertised, therefore task-level Forbid/Permit is sufficient
; for the gate update. A contender sleeps one DOS tick instead of spinning.
; ------------------------------------------------
AcquireIOGate:
        movem.l d0-d1/a0-a1/a6,-(sp)
.aig_retry:
        move.l  SysBasePtr,a6
        jsr     Forbid(a6)
        tst.b   io_gate_busy
        beq.s   .aig_take
        jsr     Permit(a6)
        move.l  DosBasePtr,a6
        moveq   #1,d1
        jsr     Delay(a6)
        bra.s   .aig_retry
.aig_take:
        move.b  #1,io_gate_busy
        move.l  SysBasePtr,a6
        jsr     Permit(a6)
        movem.l (sp)+,d0-d1/a0-a1/a6
        rts

ReleaseIOGate:
        movem.l d0/a0-a1/a6,-(sp)
        move.l  SysBasePtr,a6
        jsr     Forbid(a6)
        clr.b   io_gate_busy
        move.l  SysBasePtr,a6
        jsr     Permit(a6)
        movem.l (sp)+,d0/a0-a1/a6
        rts

DevAbortIO:
        move.b  #IOERR_ABORTED,IO_ERROR(a1)
        moveq   #IOERR_ABORTED,d0
        rts

; ================================================================
; One-time DWC2 hardware bring-up
; ================================================================
EnsureHardware:
        tst.b   hw_ready
        beq.s   EHWStart
        moveq   #0,d0
        rts

EHWStart:
        move.l  SysBasePtr,a4
        move.l  a4,a6

        ; dos.library is used only for Delay() in this development build.
        lea     DosName(pc),a1
        jsr     OldOpenLibrary(a6)
        tst.l   d0
        beq.w   EHWFail
        move.l  d0,DosBasePtr
        move.l  d0,a5

        ; Mailbox property buffer.
        move.l  #PROP_ALLOC_SIZE,d0
        move.l  #MEMF_FASTCLEAR,d1
        move.l  a4,a6
        jsr     AllocMem(a6)
        tst.l   d0
        beq.w   EHWFail
        move.l  d0,prop_alloc
        add.l   #127,d0
        and.l   #$FFFFFF80,d0
        move.l  d0,prop_ptr

        ; DMA transfer workspace.
        move.l  #DMA_ALLOC_SIZE,d0
        move.l  #MEMF_FASTCLEAR,d1
        move.l  a4,a6
        jsr     AllocMem(a6)
        tst.l   d0
        beq.w   EHWFail
        move.l  d0,dma_alloc
        add.l   #63,d0
        and.l   #$FFFFFFC0,d0
        move.l  d0,dma_base
        move.l  d0,setup_ptr
        add.l   #64,d0
        move.l  d0,data_ptr            ; 64-byte aligned 256 KiB Bulk DMA bounce area
        add.l   #BULK_DMA_MAX,d0
        move.l  d0,status_ptr

        ; Firmware USB-HCD power ON.
        bsr     build_set_power
        bsr     prop_dma_pre
        bsr     mailbox_property
        tst.l   d0
        bne.w   EHWFail
        bsr     prop_dma_post

        ; Verify DWC OTG signature.
        move.l  #GSNPSID,a0
        bsr     mmio_read32
        move.l  d0,d1
        and.l   #$FFFFF000,d1
        cmp.l   #$4F542000,d1
        beq.s   EHWIDOK
        cmp.l   #$4F543000,d1
        bne.w   EHWFail
EHWIDOK:
        bsr     init_core_host
        tst.l   d0
        bne.w   EHWFail

        ; USB57: the host controller must be operational even with an empty
        ; physical USB port.  Power the DWC2 root port, but do NOT require a
        ; connected device and do not reset it here.  Poseidon talks to our
        ; virtual one-port root hub and hub.class performs PORT_RESET only
        ; when a real device is present.
        bsr     root_port_power_only
        tst.l   d0
        bne.w   EHWFail

        clr.b   current_addr
        clr.b   root_hub_addr
        clr.b   root_hub_config
        clr.w   root_change_bits
        bsr     update_root_changes
        move.b  #1,hw_ready
        moveq   #0,d0
        rts

EHWFail:
        moveq   #1,d0
        rts

; ================================================================
; USB57 virtual one-port USB 2.0 root hub
; ================================================================
;
; Poseidon HCDs emulate their controller root hub.  This keeps the HCD itself
; online with an empty physical port and gives hub.class a permanent interrupt
; endpoint through which connect/disconnect changes can be reported.

DoRootHubControl:
        move.l  CurrentIOReq,a1
        moveq   #0,d0
        move.b  req_bm,d0
        cmpi.b  #$00,d0
        beq.w   RHStdOutDevice
        cmpi.b  #$80,d0
        beq.w   RHStdInDevice
        cmpi.b  #$23,d0
        beq.w   RHClassOutPort
        cmpi.b  #$A3,d0
        beq.w   RHClassInPort
        cmpi.b  #$A0,d0
        beq.w   RHClassInHub
        bra.w   RHStall

RHStdOutDevice:
        moveq   #0,d0
        move.b  req_b,d0
        cmpi.b  #5,d0                    ; SET_ADDRESS
        beq.s   .rh_set_address
        cmpi.b  #9,d0                    ; SET_CONFIGURATION
        beq.s   .rh_set_config
        bra.w   RHStall
.rh_set_address:
        moveq   #0,d0
        move.w  req_value,d0
        cmpi.w  #127,d0
        bhi.w   RHStall
        move.b  d0,root_hub_addr
        clr.l   UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone
.rh_set_config:
        moveq   #0,d0
        move.w  req_value,d0
        cmpi.w  #1,d0
        bhi.w   RHStall
        move.b  d0,root_hub_config
        clr.l   UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

RHStdInDevice:
        moveq   #0,d0
        move.b  req_b,d0
        beq.s   .rh_get_dev_status       ; GET_STATUS
        cmpi.b  #6,d0
        beq.s   .rh_get_descriptor       ; GET_DESCRIPTOR
        cmpi.b  #8,d0
        beq.s   .rh_get_configuration    ; GET_CONFIGURATION
        bra.w   RHStall
.rh_get_dev_status:
        move.l  UIO_DATA_BUFFER(a1),a2
        move.l  a2,d0
        beq.w   RHBadParam
        cmpi.l  #2,UIO_DATA_BUF_LEN(a1)
        bcs.w   RHBadParam
        move.b  #1,(a2)+                 ; self powered
        clr.b   (a2)
        move.l  #2,UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone
.rh_get_configuration:
        move.l  UIO_DATA_BUFFER(a1),a2
        move.l  a2,d0
        beq.w   RHBadParam
        tst.l   UIO_DATA_BUF_LEN(a1)
        beq.w   RHBadParam
        move.b  root_hub_config,(a2)
        move.l  #1,UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone
.rh_get_descriptor:
        moveq   #0,d0
        move.w  req_value,d0
        lsr.w   #8,d0
        cmpi.b  #1,d0
        beq.s   .rh_desc_device
        cmpi.b  #2,d0
        beq.s   .rh_desc_config
        bra.w   RHStall
.rh_desc_device:
        lea     RootHubDeviceDesc(pc),a0
        moveq   #18,d0
        bra.w   RHCopyDescriptor
.rh_desc_config:
        lea     RootHubConfigTree(pc),a0
        moveq   #25,d0
        bra.w   RHCopyDescriptor

RHClassInHub:
        moveq   #0,d0
        move.b  req_b,d0
        beq.s   .rh_hub_status
        cmpi.b  #6,d0
        beq.s   .rh_hub_descriptor
        bra.w   RHStall
.rh_hub_status:
        move.l  UIO_DATA_BUFFER(a1),a2
        move.l  a2,d0
        beq.w   RHBadParam
        cmpi.l  #4,UIO_DATA_BUF_LEN(a1)
        bcs.w   RHBadParam
        clr.l   (a2)
        move.l  #4,UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone
.rh_hub_descriptor:
        moveq   #0,d0
        move.w  req_value,d0
        lsr.w   #8,d0
        cmpi.b  #$29,d0
        bne.w   RHStall
        lea     RootHubHubDesc(pc),a0
        moveq   #9,d0
        bra.w   RHCopyDescriptor

RHClassInPort:
        moveq   #0,d0
        move.b  req_b,d0
        bne.w   RHStall                   ; only GET_STATUS
        cmpi.w  #1,req_index
        bne.w   RHStall
        bsr     update_root_changes
        move.l  CurrentIOReq,a1
        move.l  UIO_DATA_BUFFER(a1),a2
        move.l  a2,d0
        beq.w   RHBadParam
        cmpi.l  #4,UIO_DATA_BUF_LEN(a1)
        bcs.w   RHBadParam

        ; USB hub wPortStatus (little endian)
        moveq   #0,d0
        move.l  d6,d2
        btst    #0,d2
        beq.s   .rh_ps_no_conn
        bset    #0,d0                    ; CONNECTION
.rh_ps_no_conn:
        btst    #2,d2
        beq.s   .rh_ps_no_enable
        bset    #1,d0                    ; ENABLE
.rh_ps_no_enable:
        btst    #8,d2
        beq.s   .rh_ps_no_reset
        bset    #4,d0                    ; RESET
.rh_ps_no_reset:
        btst    #12,d2
        beq.s   .rh_ps_no_power
        bset    #8,d0                    ; POWER
.rh_ps_no_power:
        ; DWC2 PRTSPD: 0=high, 1=full, 2=low. Only meaningful connected.
        btst    #0,d2
        beq.s   .rh_ps_speed_done
        move.l  d2,d3
        and.l   #HPRT_SPD_MASK,d3
        cmpi.l  #HPRT_SPD_LOW,d3
        bne.s   .rh_ps_not_low
        bset    #9,d0                    ; LOW_SPEED
        bra.s   .rh_ps_speed_done
.rh_ps_not_low:
        tst.l   d3
        bne.s   .rh_ps_speed_done        ; full speed = neither bit
        bset    #10,d0                   ; HIGH_SPEED
.rh_ps_speed_done:
        move.b  d0,(a2)+
        lsr.w   #8,d0
        move.b  d0,(a2)+
        move.w  root_change_bits,d1
        move.b  d1,(a2)+
        lsr.w   #8,d1
        move.b  d1,(a2)
        move.l  #4,UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

RHClassOutPort:
        cmpi.w  #1,req_index
        bne.w   RHStall
        moveq   #0,d0
        move.b  req_b,d0
        cmpi.b  #3,d0                    ; SET_FEATURE
        beq.s   .rh_set_port_feature
        cmpi.b  #1,d0                    ; CLEAR_FEATURE
        beq.w   .rh_clear_port_feature
        bra.w   RHStall
.rh_set_port_feature:
        moveq   #0,d0
        move.w  req_value,d0
        cmpi.w  #RH_PORT_POWER,d0
        beq.s   .rh_set_power
        cmpi.w  #RH_PORT_RESET,d0
        beq.s   .rh_set_reset
        cmpi.w  #RH_PORT_ENABLE,d0
        beq.s   .rh_port_success
        bra.w   RHStall
.rh_set_power:
        bsr     root_port_power_only
        tst.l   d0
        bne.w   RHHCIError
        bra.s   .rh_port_success
.rh_set_reset:
        bsr     prepare_root_port
        tst.l   d0
        bne.w   RHHCIError
        ori.w   #$0010,root_change_bits   ; C_PORT_RESET
        bsr     update_root_changes
.rh_port_success:
        move.l  CurrentIOReq,a1
        clr.l   UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

.rh_clear_port_feature:
        moveq   #0,d0
        move.w  req_value,d0
        cmpi.w  #RH_C_PORT_CONNECTION,d0
        beq.s   .rh_clear_c_conn
        cmpi.w  #RH_C_PORT_ENABLE,d0
        beq.s   .rh_clear_c_enable
        cmpi.w  #RH_C_PORT_OVERCURRENT,d0
        beq.s   .rh_clear_c_over
        cmpi.w  #RH_C_PORT_RESET,d0
        beq.s   .rh_clear_c_reset
        cmpi.w  #RH_C_PORT_SUSPEND,d0
        beq.s   .rh_clear_c_suspend
        cmpi.w  #RH_PORT_POWER,d0
        beq.s   .rh_clear_power
        cmpi.w  #RH_PORT_ENABLE,d0
        beq.s   .rh_clear_enable
        bra.w   RHStall
.rh_clear_c_conn:
        andi.w  #$FFFE,root_change_bits
        move.l  #HPRT_CONNDET,d0
        bsr     root_clear_hprt_change
        bra.s   .rh_port_success
.rh_clear_c_enable:
        andi.w  #$FFFD,root_change_bits
        move.l  #HPRT_ENACHG,d0
        bsr     root_clear_hprt_change
        bra.s   .rh_port_success
.rh_clear_c_suspend:
        andi.w  #$FFFB,root_change_bits
        bra.s   .rh_port_success
.rh_clear_c_over:
        andi.w  #$FFF7,root_change_bits
        move.l  #HPRT_OVRCHG,d0
        bsr     root_clear_hprt_change
        bra.w   .rh_port_success
.rh_clear_c_reset:
        andi.w  #$FFEF,root_change_bits
        bra.w   .rh_port_success
.rh_clear_power:
        bsr     read_hprt
        move.l  d6,d0
        and.l   #HPRT_SAFE_MASK,d0
        and.l   #$FFFFEFFF,d0
        bsr     write_hprt_d0
        bra.w   .rh_port_success
.rh_clear_enable:
        ; DWC2 PRTENA is write-one-to-disable.  Preserve only safe control bits
        ; and write ENA=1 to request disable.
        bsr     read_hprt
        move.l  d6,d0
        and.l   #HPRT_SAFE_MASK,d0
        or.l    #HPRT_ENA,d0
        bsr     write_hprt_d0
        bra.w   .rh_port_success

; Copy min(descriptor-size, setup.wLength, caller buffer length) bytes.
; A0=source, D0=descriptor size.
RHCopyDescriptor:
        move.l  CurrentIOReq,a1
        move.l  UIO_DATA_BUFFER(a1),a2
        move.l  a2,d1
        beq.w   RHBadParam
        move.l  d0,d2
        moveq   #0,d3
        move.w  req_length,d3
        cmp.l   d3,d2
        bls.s   .rh_copy_req_ok
        move.l  d3,d2
.rh_copy_req_ok:
        move.l  UIO_DATA_BUF_LEN(a1),d3
        cmp.l   d3,d2
        bls.s   .rh_copy_len_ok
        move.l  d3,d2
.rh_copy_len_ok:
        move.l  d2,UIO_ACTUAL_LENGTH(a1)
        tst.l   d2
        beq.s   .rh_copy_done
.rh_copy_loop:
        move.b  (a0)+,(a2)+
        subq.l  #1,d2
        bne.s   .rh_copy_loop
.rh_copy_done:
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        bra.w   BeginIODone

RHBadParam:
        move.l  CurrentIOReq,a1
        clr.l   UIO_ACTUAL_LENGTH(a1)
        move.b  #ERR_BAD_PARAMETERS,IO_ERROR(a1)
        bra.w   BeginIODone
RHStall:
        move.l  CurrentIOReq,a1
        clr.l   UIO_ACTUAL_LENGTH(a1)
        move.b  #ERR_STALL,IO_ERROR(a1)
        bra.w   BeginIODone
RHHCIError:
        move.l  CurrentIOReq,a1
        clr.l   UIO_ACTUAL_LENGTH(a1)
        move.b  #ERR_HCI_ERROR,IO_ERROR(a1)
        bra.w   BeginIODone

; Root hub interrupt endpoint. Bit 1 reports a change on root port 1.
DoRootHubInterrupt:
        move.l  CurrentIOReq,a5
        cmpi.w  #1,UIO_ENDPOINT(a5)
        bne.w   RHIntrBad
        cmpi.w  #DIRECTION_IN,UIO_DIRECTION(a5)
        bne.w   RHIntrBad
        tst.l   UIO_DATA_BUF_LEN(a5)
        beq.w   RHIntrBad
        move.l  UIO_DATA_BUFFER(a5),a2
        move.l  a2,d0
        beq.w   RHIntrBad

        bsr     update_root_changes
        tst.w   root_change_bits
        bne.s   .rh_intr_changed

        ; No change yet. Do not occupy the single DWC2 transfer gate while
        ; hub.class waits. A short cooperative poll keeps this development HCD
        ; simple while still providing genuine plug/unplug notifications.
        bsr     ReleaseIOGate
        move.l  DosBasePtr,a6
        moveq   #2,d1
        jsr     Delay(a6)
        bsr     update_root_changes
        move.l  a5,a1
        move.l  UIO_DATA_BUFFER(a1),a2
        tst.w   root_change_bits
        bne.s   .rh_intr_changed_nogate
        clr.b   (a2)
        move.l  #1,UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        btst    #0,IO_FLAGS(a1)
        bne.s   .rh_intr_idle_done
        move.l  SysBasePtr,a6
        jsr     ReplyMsg(a6)
.rh_intr_idle_done:
        movem.l (sp)+,d1-d7/a0-a6
        rts

.rh_intr_changed:
        move.b  #2,(a2)                  ; bit1 = port 1 changed
        move.l  #1,UIO_ACTUAL_LENGTH(a5)
        clr.w   UIO_EXTERROR(a5)
        clr.b   IO_ERROR(a5)
        bra.w   BeginIODone

.rh_intr_changed_nogate:
        move.b  #2,(a2)
        move.l  #1,UIO_ACTUAL_LENGTH(a1)
        clr.w   UIO_EXTERROR(a1)
        clr.b   IO_ERROR(a1)
        btst    #0,IO_FLAGS(a1)
        bne.s   .rh_intr_changed_done
        move.l  SysBasePtr,a6
        jsr     ReplyMsg(a6)
.rh_intr_changed_done:
        movem.l (sp)+,d1-d7/a0-a6
        rts

RHIntrBad:
        move.l  CurrentIOReq,a1
        move.b  #ERR_BAD_PARAMETERS,IO_ERROR(a1)
        bra.w   BeginIODone

; Latch DWC2 root-port status/change bits into the USB hub view.
update_root_changes:
        movem.l d0-d5/a0,-(sp)
        bsr     read_hprt                  ; HPRT snapshot in D6
        move.w  root_change_bits,d5
        btst    #1,d6                      ; PRTCONNDET
        beq.s   .urc_no_connchg
        bset    #0,d5
.urc_no_connchg:
        btst    #3,d6                      ; PRTENCHNG
        beq.s   .urc_no_enachg
        bset    #1,d5
.urc_no_enachg:
        btst    #5,d6                      ; PRTOVRCURRCHNG
        beq.s   .urc_no_ovrchg
        bset    #3,d5
.urc_no_ovrchg:
        moveq   #0,d0
        btst    #0,d6
        beq.s   .urc_conn_now
        moveq   #1,d0
.urc_conn_now:
        cmp.b   root_last_conn,d0
        beq.s   .urc_conn_same
        bset    #0,d5
        move.b  d0,root_last_conn
.urc_conn_same:
        move.w  d5,root_change_bits
        movem.l (sp)+,d0-d5/a0
        rts

; D0 = HPRT change bit (W1C) to acknowledge.
root_clear_hprt_change:
        movem.l d1/d6/a0,-(sp)
        move.l  d0,d1
        bsr     read_hprt
        move.l  d6,d0
        and.l   #HPRT_SAFE_MASK,d0
        or.l    d1,d0
        bsr     write_hprt_d0
        movem.l (sp)+,d1/d6/a0
        rts

; Power the physical DWC2 root port without requiring a connected device.
root_port_power_only:
        bsr     read_hprt
        move.l  d6,d0
        and.l   #HPRT_SAFE_MASK,d0
        or.l    #HPRT_PWR,d0
        bsr     write_hprt_d0
        move.l  DosBasePtr,a6
        moveq   #2,d1
        jsr     Delay(a6)
        bsr     read_hprt
        move.l  d6,d0
        btst    #12,d0
        beq.s   .rppo_fail
        moveq   #0,d0
        rts
.rppo_fail:
        moveq   #1,d0
        rts

; ================================================================
; Generic control-transfer helpers from the proven USB17 path
; ================================================================

build_setup_packet:
        move.l  setup_ptr,a0
        move.b  req_bm,(a0)+
        move.b  req_b,(a0)+

        move.w  req_value,d0
        move.b  d0,(a0)+
        lsr.w   #8,d0
        move.b  d0,(a0)+

        move.w  req_index,d0
        move.b  d0,(a0)+
        lsr.w   #8,d0
        move.b  d0,(a0)+

        move.w  req_length,d0
        move.b  d0,(a0)+
        lsr.w   #8,d0
        move.b  d0,(a0)+
        rts

; SETUP OUT / no DATA / STATUS IN DATA1
; Used by standard SET_ADDRESS and other zero-length host-to-device requests.
control_nodata_out:
        bsr     build_setup_packet
        clr.b   ctrl_error_stage
        clr.l   ctrl_error_hcint
        clr.l   ctrl_setup_status
        clr.l   ctrl_data_status
        clr.l   ctrl_status_status

        ; SETUP OUT
        move.l  setup_ptr,a0
        moveq   #8,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_pre
        tst.l   d0
        bne.w   CNOFail1

        move.l  #HCTSIZ_SETUP_8,d0
        moveq   #0,d1
        move.l  dma_bus,d2
        bsr     channel0_start
        tst.l   d0
        bne.w   CNOFail1Post

        bsr     channel0_wait
        move.l  d0,ctrl_setup_status

        move.l  setup_ptr,a0
        moveq   #8,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_post

        move.l  ctrl_setup_status,d0
        bsr     transfer_status_ok
        tst.l   d0
        bne.w   CNOFailSetup

        ; STATUS IN, zero payload, DATA1. DWC2 still needs PKTCNT=1.
        move.l  status_ptr,a0
        moveq   #4,d0
        moveq   #0,d1
        bsr     dma_pre
        tst.l   d0
        bne.w   CNOFail2

        move.l  #HCTSIZ_DATA1_BASE,d0
        moveq   #1,d1
        move.l  dma_bus,d2
        bsr     channel0_start
        tst.l   d0
        bne.w   CNOFail2Post

        bsr     channel0_wait
        move.l  d0,ctrl_status_status

        move.l  status_ptr,a0
        moveq   #4,d0
        moveq   #0,d1
        bsr     dma_post

        move.l  ctrl_status_status,d0
        bsr     transfer_status_ok
        tst.l   d0
        bne.w   CNOFailStatus

        moveq   #0,d0
        rts

CNOFail1:
        move.b  #1,ctrl_error_stage
        moveq   #1,d0
        rts
CNOFail1Post:
        move.l  setup_ptr,a0
        moveq   #8,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_post
        move.b  #1,ctrl_error_stage
        moveq   #1,d0
        rts
CNOFailSetup:
        move.b  #1,ctrl_error_stage
        move.l  ctrl_setup_status,ctrl_error_hcint
        moveq   #1,d0
        rts
CNOFail2:
        move.b  #3,ctrl_error_stage
        moveq   #1,d0
        rts
CNOFail2Post:
        move.l  status_ptr,a0
        moveq   #4,d0
        moveq   #0,d1
        bsr     dma_post
        move.b  #3,ctrl_error_stage
        moveq   #1,d0
        rts
CNOFailStatus:
        move.b  #3,ctrl_error_stage
        move.l  ctrl_status_status,ctrl_error_hcint
        moveq   #1,d0
        rts

; SETUP OUT / DATA IN DATA1 / STATUS OUT DATA1
control_in:
        bsr     build_setup_packet
        clr.b   ctrl_error_stage
        clr.l   ctrl_error_hcint
        clr.l   ctrl_setup_status
        clr.l   ctrl_data_status
        clr.l   ctrl_status_status

        ; Clear first 64 bytes.
        move.l  data_ptr,a0
        moveq   #63,d7
CIFill:
        move.b  #$A5,(a0)+
        dbra    d7,CIFill

        ; SETUP
        move.l  setup_ptr,a0
        moveq   #8,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_pre
        tst.l   d0
        bne.w   CIFail1

        move.l  #HCTSIZ_SETUP_8,d0
        moveq   #0,d1
        move.l  dma_bus,d2
        bsr     channel0_start
        tst.l   d0
        bne.w   CIFail1Post

        bsr     channel0_wait
        move.l  d0,ctrl_setup_status

        move.l  setup_ptr,a0
        moveq   #8,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_post

        move.l  ctrl_setup_status,d0
        bsr     transfer_status_ok
        tst.l   d0
        bne.w   CIFailSetup

        ; DATA IN
        move.l  data_ptr,a0
        moveq   #0,d0
        move.w  req_length,d0
        moveq   #0,d1
        bsr     dma_pre
        tst.l   d0
        bne.w   CIFail2

        ; USB60: HCTSIZ packet count must match EP0 MPS for FS/LS devices.
        ; For the proven V57 HS case (len<=64, MPS64) this still produces
        ; exactly DATA1 + PKTCNT1 + length.
        moveq   #0,d0
        move.w  req_length,d0             ; transfer size
        move.l  d0,d4
        moveq   #0,d5
        move.w  ctrl_mps,d5
        move.l  d4,d6
        add.l   d5,d6
        subq.l  #1,d6
        divu.w  d5,d6                     ; quotient = ceil(length/MPS)
        and.l   #$0000FFFF,d6
        lsl.l   #8,d6
        lsl.l   #8,d6
        lsl.l   #3,d6                     ; packet count << 19
        or.l    d6,d0
        or.l    #HCTSIZ_PID_DATA1,d0
        moveq   #1,d1
        move.l  dma_bus,d2
        bsr     channel0_start
        tst.l   d0
        bne.w   CIFail2Post

        bsr     channel0_wait
        move.l  d0,ctrl_data_status

        move.l  data_ptr,a0
        moveq   #0,d0
        move.w  req_length,d0
        moveq   #0,d1
        bsr     dma_post

        move.l  ctrl_data_status,d0
        bsr     transfer_status_ok
        tst.l   d0
        bne.w   CIFailData

        ; STATUS OUT, zero payload.
        move.l  status_ptr,a0
        moveq   #4,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_pre
        tst.l   d0
        bne.w   CIFail3

        move.l  #HCTSIZ_DATA1_BASE,d0
        moveq   #0,d1
        move.l  dma_bus,d2
        bsr     channel0_start
        tst.l   d0
        bne.w   CIFail3Post

        bsr     channel0_wait
        move.l  d0,ctrl_status_status

        move.l  status_ptr,a0
        moveq   #4,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_post

        move.l  ctrl_status_status,d0
        bsr     transfer_status_ok
        tst.l   d0
        bne.w   CIFailStatus

        moveq   #0,d0
        rts

CIFail1:
        move.b  #1,ctrl_error_stage
        moveq   #1,d0
        rts
CIFail1Post:
        move.l  setup_ptr,a0
        moveq   #8,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_post
        move.b  #1,ctrl_error_stage
        moveq   #1,d0
        rts
CIFailSetup:
        move.b  #1,ctrl_error_stage
        move.l  ctrl_setup_status,ctrl_error_hcint
        moveq   #1,d0
        rts
CIFail2:
        move.b  #2,ctrl_error_stage
        moveq   #1,d0
        rts
CIFail2Post:
        move.l  data_ptr,a0
        moveq   #0,d0
        move.w  req_length,d0
        moveq   #0,d1
        bsr     dma_post
        move.b  #2,ctrl_error_stage
        moveq   #1,d0
        rts
CIFailData:
        move.b  #2,ctrl_error_stage
        move.l  ctrl_data_status,ctrl_error_hcint
        moveq   #1,d0
        rts
CIFail3:
        move.b  #3,ctrl_error_stage
        moveq   #1,d0
        rts
CIFail3Post:
        move.l  status_ptr,a0
        moveq   #4,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_post
        move.b  #3,ctrl_error_stage
        moveq   #1,d0
        rts
CIFailStatus:
        move.b  #3,ctrl_error_stage
        move.l  ctrl_status_status,ctrl_error_hcint
        moveq   #1,d0
        rts

; ================================================================
; USB66 split EP0 engine
; Only called when Poseidon sets UHFF_SPLITTRANS. The USB64 direct EP0 path is
; deliberately untouched. Split data-IN is issued one max-packet transaction at
; a time because each downstream FS/LS transaction needs its own SSPLIT/CSPLIT.
; ================================================================
control_split_request:
        bsr     build_setup_packet
        clr.b   ctrl_error_stage
        clr.l   ctrl_error_hcint
        clr.l   ctrl_setup_status
        clr.l   ctrl_data_status
        clr.l   ctrl_status_status
        clr.l   ctrl_actual

        ; SETUP OUT
        move.l  setup_ptr,a0
        moveq   #8,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_pre
        tst.l   d0
        bne.w   CSRFailSetupDMA
        move.l  #HCTSIZ_SETUP_8,d0
        moveq   #0,d1
        move.l  dma_bus,d2
        bsr     channel0_run_split_control
        move.l  d0,ctrl_setup_status
        move.l  setup_ptr,a0
        moveq   #8,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_post
        move.l  ctrl_setup_status,d0
        bsr     transfer_status_ok
        tst.l   d0
        bne.w   CSRFailSetup

        moveq   #0,d6
        move.w  req_length,d6
        beq.w   CSRStatusIn

        ; USB66 focuses on enumeration/HID reads: split data phase must be IN.
        moveq   #0,d0
        move.b  req_bm,d0
        btst    #7,d0
        beq.w   CSRFailBadDataDir

        clr.l   ctrl_data_offset
        move.l  #HCTSIZ_PID_DATA1,ctrl_data_pid

CSRDataLoop:
        moveq   #0,d5
        move.w  ctrl_mps,d5
        move.l  ctrl_data_offset,d0
        moveq   #0,d1
        move.w  req_length,d1
        sub.l   d0,d1                    ; remaining bytes
        cmp.l   d5,d1
        bcc.s   .csr_chunk_ready
        move.l  d1,d5
.csr_chunk_ready:
        move.l  d5,ctrl_chunk_len          ; caller still expects at most this many bytes

        ; USB69 / DWC2 split-IN rule: HCTSIZ.XferSize and the DMA reservation
        ; must be a *full endpoint max packet* for every split IN transaction,
        ; even when only a short tail (for example bytes 17..18 of an 18-byte
        ; descriptor with MPS=8) remains.  The device terminates naturally with
        ; a short packet and residual XferSize tells us how many bytes arrived.
        ; Linux dwc2_hc_start_transfer() applies the same rule for do_split+IN.
        moveq   #0,d6
        move.w  ctrl_mps,d6
        move.l  d6,ctrl_split_dma_len

        move.l  data_ptr,a0
        add.l   ctrl_data_offset,a0
        move.l  d6,d0
        moveq   #0,d1                    ; device -> RAM
        bsr     dma_pre
        tst.l   d0
        bne.w   CSRFailDataDMA

        move.l  ctrl_split_dma_len,d0    ; full MPS reservation, not short tail
        or.l    #HCTSIZ_PKTCNT_1,d0
        or.l    ctrl_data_pid,d0
        moveq   #1,d1                    ; IN
        move.l  dma_bus,d2
        bsr     channel0_run_split_control
        move.l  d0,ctrl_data_status
        move.l  #HCTSIZ0,a0
        bsr     mmio_read32
        move.l  d0,ctrl_final_hctsiz

        move.l  data_ptr,a0
        add.l   ctrl_data_offset,a0
        move.l  ctrl_split_dma_len,d0
        moveq   #0,d1
        bsr     dma_post

        move.l  ctrl_data_status,d0
        bsr     transfer_status_ok
        tst.l   d0
        bne.w   CSRFailData

        ; actual = full-MPS reservation - residual xfersize
        move.l  ctrl_final_hctsiz,d0
        and.l   #$0007FFFF,d0
        cmp.l   ctrl_split_dma_len,d0
        bhi.w   CSRFailDataLength
        move.l  ctrl_split_dma_len,d4
        sub.l   d0,d4
        ; A device must never return beyond the remaining setup wLength.
        cmp.l   ctrl_chunk_len,d4
        bhi.w   CSRFailDataLength
        add.l   d4,ctrl_actual
        add.l   d4,ctrl_data_offset

        ; USB70: advance the control DATA PID explicitly after each successful
        ; downstream packet.  Split EP0 DATA always starts at DATA1 and then
        ; alternates DATA1/DATA0/DATA1... packet-by-packet.  Do not derive the
        ; next PID from HCTSIZ after a CSPLIT: on this DWC2 path that field is
        ; not a reliable software-visible next-toggle source across separately
        ; re-armed SSPLIT/CSPLIT transactions.
        move.l  ctrl_data_pid,d0
        cmpi.l  #HCTSIZ_PID_DATA1,d0
        bne.s   .csr_next_data1
        clr.l   ctrl_data_pid             ; DATA0 = 0 in HCTSIZ PID field
        bra.s   .csr_pid_advanced
.csr_next_data1:
        move.l  #HCTSIZ_PID_DATA1,ctrl_data_pid
.csr_pid_advanced:

        ; Any packet shorter than MPS terminates the USB control data stage.
        cmp.l   ctrl_split_dma_len,d4
        bcs.s   CSRDataDone
        moveq   #0,d0
        move.w  req_length,d0
        cmp.l   ctrl_data_offset,d0
        bhi.w   CSRDataLoop
CSRDataDone:
        ; IN data stage -> STATUS OUT DATA1
        move.l  status_ptr,a0
        moveq   #4,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_pre
        tst.l   d0
        bne.w   CSRFailStatusDMA
        move.l  #HCTSIZ_DATA1_BASE,d0
        moveq   #0,d1
        move.l  dma_bus,d2
        bsr     channel0_run_split_control
        move.l  d0,ctrl_status_status
        move.l  status_ptr,a0
        moveq   #4,d0
        move.l  #DMA_ReadFromRAM,d1
        bsr     dma_post
        bra.s   CSRCheckStatus

CSRStatusIn:
        ; no-data host-to-device -> STATUS IN DATA1. For split IN, DWC2 also
        ; wants a full-MPS receive reservation although the USB status packet
        ; itself is a ZLP. This is the same host-DMA rule as split DATA-IN.
        moveq   #0,d6
        move.w  ctrl_mps,d6
        move.l  status_ptr,a0
        move.l  d6,d0
        moveq   #0,d1
        bsr     dma_pre
        tst.l   d0
        bne.w   CSRFailStatusDMA
        move.l  d6,d0
        or.l    #HCTSIZ_PKTCNT_1+HCTSIZ_PID_DATA1,d0
        moveq   #1,d1
        move.l  dma_bus,d2
        bsr     channel0_run_split_control
        move.l  d0,ctrl_status_status
        move.l  status_ptr,a0
        moveq   #0,d0
        move.w  ctrl_mps,d0
        moveq   #0,d1
        bsr     dma_post

CSRCheckStatus:
        move.l  ctrl_status_status,d0
        bsr     transfer_status_ok
        tst.l   d0
        bne.w   CSRFailStatus
        moveq   #0,d0
        rts

CSRFailSetupDMA:
        move.b  #1,ctrl_error_stage
        moveq   #1,d0
        rts
CSRFailSetup:
        move.b  #1,ctrl_error_stage
        move.l  ctrl_setup_status,ctrl_error_hcint
        moveq   #1,d0
        rts
CSRFailBadDataDir:
        move.b  #2,ctrl_error_stage
        move.l  #$00007FFC,ctrl_error_hcint
        moveq   #1,d0
        rts
CSRFailDataDMA:
        move.b  #2,ctrl_error_stage
        moveq   #1,d0
        rts
CSRFailData:
        move.b  #2,ctrl_error_stage
        move.l  ctrl_data_status,ctrl_error_hcint
        moveq   #1,d0
        rts
CSRFailDataLength:
        move.b  #2,ctrl_error_stage
        move.l  #$00007FFD,ctrl_error_hcint
        moveq   #1,d0
        rts
CSRFailStatusDMA:
        move.b  #3,ctrl_error_stage
        moveq   #1,d0
        rts
CSRFailStatus:
        move.b  #3,ctrl_error_stage
        move.l  ctrl_status_status,ctrl_error_hcint
        moveq   #1,d0
        rts

; Start one control stage with HCSPLT routing active.
; D0=HCTSIZ, D1=direction (1 IN / 0 OUT), D2=DMA bus.
channel0_start_split_control:
        move.l  d0,xfer_hctsiz
        move.l  d1,xfer_dir
        move.l  d2,xfer_bus
        move.l  #$00020000,d7
.c65c_wait:
        move.l  #HCCHAR0,a0
        bsr     mmio_read32
        btst    #31,d0
        beq.s   .c65c_free
        subq.l  #1,d7
        bne.s   .c65c_wait
        moveq   #1,d0
        rts
.c65c_free:
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCINTMSK0,a0
        bsr     mmio_write32

        moveq   #0,d0
        move.w  ctrl_split_port,d0
        and.l   #HCSPLT_PRTADDR_MASK,d0
        move.l  d0,d6
        moveq   #0,d0
        move.w  ctrl_split_hub,d0
        lsl.l   #HCSPLT_HUBADDR_SHIFT,d0
        or.l    d0,d6
        or.l    #HCSPLT_XACTPOS_ALL+HCSPLT_SPLTENA,d6
        tst.b   split_complete
        beq.s   .c65c_split_ready
        or.l    #HCSPLT_COMPSPLT,d6
.c65c_split_ready:
        move.l  d6,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32

        move.l  xfer_hctsiz,d0
        move.l  #HCTSIZ0,a0
        bsr     mmio_write32
        move.l  xfer_bus,d0
        move.l  #HCDMA0,a0
        bsr     mmio_write32

        moveq   #0,d6
        move.w  ctrl_mps,d6
        or.l    #HCCHAR_MULTICNT_1,d6
        move.w  ctrl_flags,d0
        btst    #0,d0
        beq.s   .c65c_not_low
        or.l    #HCCHAR_LSDEV,d6
.c65c_not_low:
        tst.l   xfer_dir
        beq.s   .c65c_dir_done
        or.l    #HCCHAR_EPDIR_IN,d6
.c65c_dir_done:
        moveq   #0,d0
        move.b  current_addr,d0
        lsl.l   #8,d0
        lsl.l   #8,d0
        lsl.l   #6,d0
        or.l    d0,d6
        move.l  #HFNUM,a0
        bsr     mmio_read32
        btst    #0,d0
        beq.s   .c65c_even
        or.l    #HCCHAR_ODDFRM,d6
.c65c_even:
        move.l  d6,d0
        or.l    #HCCHAR_CHENA,d0
        and.l   #$BFFFFFFF,d0
        move.l  #HCCHAR0,a0
        bsr     mmio_write32
        moveq   #0,d0
        rts

; SSPLIT followed by CSPLIT. NYET means the TT has not completed the downstream
; transaction yet; retry the complete split on a later host frame/microframe.
channel0_run_split_control:
        ; USB69: execute one *downstream* control transaction (SETUP, one DATA
        ; packet, or STATUS) through the HS hub transaction translator.
        ;
        ; Important DWC2/TT semantics:
        ;   SSPLIT NAK  -> hub did not accept the start; retry SSPLIT later.
        ;   SSPLIT ACK  -> hub accepted it; move to CSPLIT.
        ;   CSPLIT NYET -> TT is still working; retry CSPLIT (non-periodic
        ;                  control transfers are retried immediately, matching
        ;                  the reference DWC2 HCD behaviour).
        ;   CSPLIT NAK  -> downstream device NAKed; restart with a new SSPLIT.
        ;   XFERCOMP    -> this split transaction is complete.
        ;
        ; USB67 incorrectly treated CSPLIT NAK as a terminal HCD error.  That
        ; is especially visible on SET_ADDRESS's STATUS-IN phase: the first
        ; GET_DESCRIPTOR can succeed, then SET_ADDRESS dies and leaves the
        ; following enumeration attempts in a bad state.
        move.l  d0,xfer_hctsiz
        move.l  d1,xfer_dir
        move.l  d2,xfer_bus
        moveq   #64,d3                    ; whole-stage SSPLIT restart budget

.c68_ssplit_retry:
        clr.b   split_complete
        ; Never inherit a CSPLIT or stale interrupt state from the previous
        ; attempt of this same downstream transaction.
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32

        move.l  xfer_hctsiz,d0
        move.l  xfer_dir,d1
        move.l  xfer_bus,d2
        bsr     channel0_start_split_control
        tst.l   d0
        bne.w   .c68_startbad
        bsr     channel0_wait
        move.l  d0,d5

        ; Hard errors/timeouts are not protocol retries.
        btst    #31,d5
        bne.w   .c68_return
        move.l  d5,d0
        and.l   #HCINT_ERROR_MASK,d0
        bne.w   .c68_return
        btst    #3,d5                     ; STALL
        bne.w   .c68_return

        ; A busy TT may NAK/NYET the Start-Split.  This is not a device error;
        ; retry the same SSPLIT on a later host (micro)frame.
        btst    #4,d5                     ; NAK
        bne.w   .c68_retry_ssplit
        btst    #6,d5                     ; defensive: NYET on SSPLIT
        bne.w   .c68_retry_ssplit

        ; The normal SSPLIT completion is ACK.  Do not mistake CHHLTD alone
        ; for acceptance by the hub.
        btst    #5,d5                     ; ACK
        beq.w   .c68_return

        ; USB72: never launch the first CSPLIT in the same HS microframe as
        ; the accepted Start-Split. HFNUM advances every 125 us while the
        ; physical root partner is high-speed, so one HFNUM step gives the TT
        ; a real downstream scheduling window before we ask for completion.
        bsr     bulk_wait_next_frame

        moveq   #64,d4                    ; CSPLIT retry budget
.c68_csplit_retry:
        ; HCINT is W1C. Clear SSPLIT ACK/CHHLTD or the previous CSPLIT's NYET
        ; before arming the next Complete-Split.
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        move.b  #1,split_complete

        move.l  xfer_hctsiz,d0
        ; Complete-Split OUT has no payload. Preserve packet count/PID while
        ; clearing XferSize[18:0], as required by DWC2.
        tst.l   xfer_dir
        bne.s   .c68_csplit_hctsiz_ready
        and.l   #$FFF80000,d0
.c68_csplit_hctsiz_ready:
        move.l  xfer_dir,d1
        move.l  xfer_bus,d2
        bsr     channel0_start_split_control
        tst.l   d0
        bne.w   .c68_startbad
        bsr     channel0_wait
        move.l  d0,d5

        btst    #31,d5
        bne.w   .c68_return
        move.l  d5,d0
        and.l   #HCINT_ERROR_MASK,d0
        bne.w   .c68_return
        btst    #3,d5                     ; STALL
        bne.w   .c68_return

        ; Success wins even if informational bits are also present.
        btst    #0,d5                     ; XFERCOMP
        bne.w   .c68_return

        ; For non-periodic CONTROL CSPLIT, Linux DWC2 re-does NYET immediately.
        ; Waiting a whole frame here (USB67) can miss the valid CSPLIT window.
        btst    #6,d5                     ; NYET
        beq.s   .c68_check_csplit_nak
        subq.l  #1,d4
        beq.w   .c68_return
        ; USB72: do not hammer the same TT in one microframe.  Wait for the
        ; next HS HFNUM slot before polling the Complete-Split again.
        bsr     bulk_wait_next_frame
        bra.w   .c68_csplit_retry

.c68_check_csplit_nak:
        ; A NAK from a Complete-Split is the downstream device's NAK.  The
        ; next attempt must start a fresh SSPLIT; returning it as HOSTERROR is
        ; wrong and was the likely SET_ADDRESS failure in USB67.
        btst    #4,d5                     ; NAK
        bne.w   .c68_retry_ssplit

        ; Anything else is unexpected; return the raw HCINT for diagnostics.
        bra.w   .c68_return

.c68_retry_ssplit:
        subq.l  #1,d3
        beq.s   .c68_return
        ; Restarting a Start-Split is periodic only at the HS bus level.  Give
        ; the TT one scheduling step before trying again so we don't hammer the
        ; same busy slot.
        bsr     bulk_wait_next_frame
        bra.w   .c68_ssplit_retry

.c68_return:
        clr.b   split_complete
        ; Preserve the result before cleanup.  On hard channel errors/timeouts
        ; force a halt; a clean protocol halt (NAK/NYET/ACK/XFERCOMP) does not
        ; need the expensive recovery sequence.
        move.l  d5,d0
        btst    #31,d0
        bne.s   .c68_recover
        move.l  d0,d6
        and.l   #HCINT_ERROR_MASK,d6
        beq.s   .c68_clean
.c68_recover:
        bsr     channel0_recover_timeout
.c68_clean:
        ; Always retire split routing and stale interrupt state before the next
        ; root-hub/HID/Bulk request touches the single physical host channel.
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32
        move.l  d5,d0
        rts

.c68_startbad:
        clr.b   split_complete
        bsr     channel0_recover_timeout
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32
        move.l  #$80000000,d0
        rts

; ================================================================
; DWC2 host channel 0
; ================================================================

channel0_start:
        move.l  d0,xfer_hctsiz
        move.l  d1,xfer_dir
        move.l  d2,xfer_bus

        move.l  #$00020000,d7
C0WaitFree:
        move.l  #HCCHAR0,a0
        bsr     mmio_read32
        btst    #31,d0
        beq.s   C0Free
        subq.l  #1,d7
        bne.s   C0WaitFree
        moveq   #1,d0
        rts
C0Free:
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32

        moveq   #0,d0
        move.l  #HCINTMSK0,a0
        bsr     mmio_write32

        moveq   #0,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32

        move.l  xfer_hctsiz,d0
        move.l  #HCTSIZ0,a0
        bsr     mmio_write32

        move.l  xfer_bus,d0
        move.l  #HCDMA0,a0
        bsr     mmio_write32

        ; USB60: use endpoint-0 MPS supplied by Poseidon.  High-speed V57
        ; traffic remains MPS64 and therefore programs the same HCCHAR value.
        moveq   #0,d6
        move.w  ctrl_mps,d6
        or.l    #HCCHAR_MULTICNT_1,d6
        move.w  ctrl_flags,d0
        btst    #0,d0                     ; UHFF_LOWSPEED
        beq.s   .c0_not_low_speed
        or.l    #HCCHAR_LSDEV,d6
.c0_not_low_speed:

        tst.l   xfer_dir
        beq.s   C0DirDone
        or.l    #HCCHAR_EPDIR_IN,d6
C0DirDone:
        moveq   #0,d0
        move.b  current_addr,d0
        lsl.l   #8,d0
        lsl.l   #8,d0
        lsl.l   #6,d0
        or.l    d0,d6

        move.l  #HFNUM,a0
        bsr     mmio_read32
        btst    #0,d0
        beq.s   C0Even
        or.l    #HCCHAR_ODDFRM,d6
C0Even:
        move.l  d6,d0
        or.l    #HCCHAR_CHENA,d0
        and.l   #$BFFFFFFF,d0
        move.l  #HCCHAR0,a0
        bsr     mmio_write32
        moveq   #0,d0
        rts

; Start high-speed BULK transfer on host channel 0.
; d0 = HCTSIZ, d2 = DMA bus address; endpoint/direction/MPS from bulk_* state.
channel0_start_bulk:
        move.l  d0,xfer_hctsiz
        move.l  d2,xfer_bus

        move.l  #$00020000,d7
C0BWaitFree:
        move.l  #HCCHAR0,a0
        bsr     mmio_read32
        btst    #31,d0
        beq.s   C0BFree
        subq.l  #1,d7
        bne.s   C0BWaitFree
        moveq   #1,d0
        rts
C0BFree:
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCINTMSK0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32

        move.l  xfer_hctsiz,d0
        move.l  #HCTSIZ0,a0
        bsr     mmio_write32
        move.l  xfer_bus,d0
        move.l  #HCDMA0,a0
        bsr     mmio_write32

        ; HCCHAR: MPS | EPNum | Bulk | EC=1 | direction | device address.
        moveq   #0,d6
        move.w  bulk_mps,d6
        moveq   #0,d0
        move.w  bulk_endpoint,d0
        lsl.l   #8,d0
        lsl.l   #3,d0              ; endpoint << 11
        or.l    d0,d6
        or.l    #HCCHAR_EPTYPE_BULK+HCCHAR_MULTICNT_1,d6
        cmpi.w  #DIRECTION_IN,bulk_direction
        bne.s   .c0b_dir_done
        or.l    #HCCHAR_EPDIR_IN,d6
.c0b_dir_done:
        moveq   #0,d0
        move.b  current_addr,d0
        lsl.l   #8,d0
        lsl.l   #8,d0
        lsl.l   #6,d0              ; device address << 22
        or.l    d0,d6

        move.l  #HFNUM,a0
        bsr     mmio_read32
        btst    #0,d0
        beq.s   .c0b_even
        or.l    #HCCHAR_ODDFRM,d6
.c0b_even:
        move.l  d6,d0
        or.l    #HCCHAR_CHENA,d0
        and.l   #$BFFFFFFF,d0
        move.l  #HCCHAR0,a0
        bsr     mmio_write32
        moveq   #0,d0
        rts

; USB57: recover host channel 0 after a software polling timeout.
; DWC2 channel halt is requested by setting CHDIS together with CHENA.
; This prevents a timed-out periodic request from poisoning the following
; hub or mass-storage transaction. Best-effort cleanup; callers decide
; whether the original condition is benign or fatal.
channel0_recover_timeout:
        movem.l d0-d3/a0,-(sp)
        move.l  #HCCHAR0,a0
        bsr     mmio_read32
        btst    #31,d0
        beq.s   .c0rt_clear
        or.l    #HCCHAR_CHDIS+HCCHAR_CHENA,d0
        move.l  #HCCHAR0,a0
        bsr     mmio_write32
        move.l  #TIMEOUT,d3
.c0rt_wait:
        move.l  #HCCHAR0,a0
        bsr     mmio_read32
        btst    #31,d0
        beq.s   .c0rt_clear
        subq.l  #1,d3
        bne.s   .c0rt_wait
.c0rt_clear:
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCINTMSK0,a0
        bsr     mmio_write32
        movem.l (sp)+,d0-d3/a0
        rts

; Start high-speed interrupt IN transfer on host channel 0.
; d0 = HCTSIZ, d2 = DMA bus address; endpoint/MPS from intr_* state.
channel0_start_interrupt:
        move.l  d0,xfer_hctsiz
        move.l  d2,xfer_bus

        move.l  #$00020000,d7
C0IWaitFree:
        move.l  #HCCHAR0,a0
        bsr     mmio_read32
        btst    #31,d0
        beq.s   C0IFree
        subq.l  #1,d7
        bne.s   C0IWaitFree
        moveq   #1,d0
        rts
C0IFree:
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCINTMSK0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32

        move.l  xfer_hctsiz,d0
        move.l  #HCTSIZ0,a0
        bsr     mmio_write32
        move.l  xfer_bus,d0
        move.l  #HCDMA0,a0
        bsr     mmio_write32

        ; HCCHAR: MPS | EPNum | Interrupt | EC=1 | IN | device address.
        moveq   #0,d6
        move.w  intr_mps,d6
        moveq   #0,d0
        move.w  intr_endpoint,d0
        lsl.l   #8,d0
        lsl.l   #3,d0
        or.l    d0,d6
        or.l    #HCCHAR_EPTYPE_INTR+HCCHAR_MULTICNT_1+HCCHAR_EPDIR_IN,d6
        move.w  intr_flags,d0
        btst    #0,d0                     ; UHFF_LOWSPEED
        beq.s   .c0i_not_low_speed
        or.l    #HCCHAR_LSDEV,d6
.c0i_not_low_speed:
        moveq   #0,d0
        move.b  current_addr,d0
        lsl.l   #8,d0
        lsl.l   #8,d0
        lsl.l   #6,d0
        or.l    d0,d6

        ; For a periodic transfer schedule the next frame parity, matching
        ; DWC2's standard even/odd-frame rule.
        move.l  #HFNUM,a0
        bsr     mmio_read32
        btst    #0,d0
        bne.s   .c0i_next_even
        or.l    #HCCHAR_ODDFRM,d6
.c0i_next_even:
        move.l  d6,d0
        or.l    #HCCHAR_CHENA,d0
        and.l   #$BFFFFFFF,d0
        move.l  #HCCHAR0,a0
        bsr     mmio_write32
        moveq   #0,d0
        rts

; USB66 downstream interrupt-IN start with HCSPLT routing.
channel0_start_split_interrupt:
        move.l  d0,xfer_hctsiz
        move.l  d2,xfer_bus
        move.l  #$00020000,d7
.c65i_wait:
        move.l  #HCCHAR0,a0
        bsr     mmio_read32
        btst    #31,d0
        beq.s   .c65i_free
        subq.l  #1,d7
        bne.s   .c65i_wait
        moveq   #1,d0
        rts
.c65i_free:
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCINTMSK0,a0
        bsr     mmio_write32

        moveq   #0,d0
        move.w  intr_split_port,d0
        and.l   #HCSPLT_PRTADDR_MASK,d0
        move.l  d0,d6
        moveq   #0,d0
        move.w  intr_split_hub,d0
        lsl.l   #HCSPLT_HUBADDR_SHIFT,d0
        or.l    d0,d6
        or.l    #HCSPLT_XACTPOS_ALL+HCSPLT_SPLTENA,d6
        tst.b   split_complete
        beq.s   .c65i_split_ready
        or.l    #HCSPLT_COMPSPLT,d6
.c65i_split_ready:
        move.l  d6,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32

        move.l  xfer_hctsiz,d0
        move.l  #HCTSIZ0,a0
        bsr     mmio_write32
        move.l  xfer_bus,d0
        move.l  #HCDMA0,a0
        bsr     mmio_write32

        moveq   #0,d6
        move.w  intr_mps,d6
        moveq   #0,d0
        move.w  intr_endpoint,d0
        lsl.l   #8,d0
        lsl.l   #3,d0
        or.l    d0,d6
        ; DWC2 split periodic endpoints use EC/MC=3 for immediate retries.
        or.l    #HCCHAR_EPTYPE_INTR+HCCHAR_MULTICNT_3+HCCHAR_EPDIR_IN,d6
        move.w  intr_flags,d0
        btst    #0,d0
        beq.s   .c65i_not_low
        or.l    #HCCHAR_LSDEV,d6
.c65i_not_low:
        moveq   #0,d0
        move.b  current_addr,d0
        lsl.l   #8,d0
        lsl.l   #8,d0
        lsl.l   #6,d0
        or.l    d0,d6
        move.l  #HFNUM,a0
        bsr     mmio_read32
        btst    #0,d0
        bne.s   .c65i_next_even
        or.l    #HCCHAR_ODDFRM,d6
.c65i_next_even:
        move.l  d6,d0
        or.l    #HCCHAR_CHENA,d0
        and.l   #$BFFFFFFF,d0
        move.l  #HCCHAR0,a0
        bsr     mmio_write32
        moveq   #0,d0
        rts

channel0_run_split_interrupt:
        ; USB73: periodic split-IN scheduling.  Unlike non-periodic CONTROL,
        ; INTERRUPT splits have a strict high-speed microframe schedule.  The
        ; DWC2/Linux scheduler advances two uframes immediately after the
        ; Start-Split, then one uframe for each following Complete-Split.
        ; USB73 accounts for the fact that HCCHAR.ODDFRM itself targets the
        ; next microframe: one explicit HFNUM advance + channel arm = +2.
        move.l  d0,intr_hctsiz
        move.l  d2,xfer_bus
        moveq   #64,d3                    ; whole interrupt SSPLIT retry budget

.c72ri_ssplit:
        clr.b   split_complete
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32

        move.l  intr_hctsiz,d0
        move.l  xfer_bus,d2
        bsr     channel0_start_split_interrupt
        tst.l   d0
        bne.w   .c72ri_startbad
        bsr     channel0_wait
        move.l  d0,d5

        btst    #31,d5
        bne.w   .c72ri_return
        btst    #3,d5                     ; STALL is terminal
        bne.w   .c72ri_return
        ; Linux DWC2 reschedules periodic split interrupt transfers after a
        ; transaction error / frame overrun instead of exposing it immediately.
        btst    #7,d5                     ; XACTERR
        bne.w   .c72ri_retry_ssplit
        btst    #9,d5                     ; FRMOVRUN
        bne.w   .c72ri_retry_ssplit
        move.l  d5,d0
        and.l   #HCINT_ERROR_MASK,d0
        bne.w   .c72ri_return

        ; A busy TT may reject the periodic Start-Split.  Retry later without
        ; changing the endpoint DATA toggle.
        btst    #4,d5                     ; NAK
        bne.w   .c72ri_retry_ssplit
        btst    #6,d5                     ; defensive NYET
        bne.w   .c72ri_retry_ssplit
        btst    #5,d5                     ; ACK = TT accepted SSPLIT
        beq.w   .c72ri_return

        ; USB73: one explicit HFNUM advance here. channel0_start_split_interrupt
        ; then arms ODDFRM for the *next* microframe.  Together this places the
        ; CSPLIT two HS microframes after the SSPLIT without overshooting by one.
        bsr     bulk_wait_next_frame

        ; INTERRUPT IN gets up to three Complete-Split opportunities in the
        ; current full-speed frame.  A NYET advances one microframe.
        moveq   #3,d4
.c72ri_csplit:
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        move.b  #1,split_complete
        move.l  intr_hctsiz,d0
        move.l  xfer_bus,d2
        bsr     channel0_start_split_interrupt
        tst.l   d0
        bne.w   .c72ri_startbad
        bsr     channel0_wait
        move.l  d0,d5

        btst    #31,d5
        bne.w   .c72ri_return
        btst    #3,d5                     ; STALL is terminal
        bne.w   .c72ri_return
        ; Linux DWC2 reschedules periodic split interrupt transfers after a
        ; transaction error / frame overrun instead of exposing it immediately.
        btst    #7,d5                     ; XACTERR
        bne.w   .c72ri_retry_ssplit
        btst    #9,d5                     ; FRMOVRUN
        bne.w   .c72ri_retry_ssplit
        move.l  d5,d0
        and.l   #HCINT_ERROR_MASK,d0
        bne.w   .c72ri_return
        btst    #0,d5                     ; XFERCOMP
        bne.w   .c72ri_return

        btst    #6,d5                     ; NYET: TT not ready yet
        beq.s   .c72ri_check_nak
        subq.l  #1,d4
        beq.w   .c72ri_return
        bsr     bulk_wait_next_frame
        bra.w   .c72ri_csplit

.c72ri_check_nak:
        ; Downstream interrupt endpoint NAK means no report is ready.  Return
        ; NAK to the existing USB64 HID idle logic rather than manufacturing
        ; a host error or advancing DATA0/DATA1.
        btst    #4,d5
        bne.w   .c72ri_return
        bra.w   .c72ri_return

.c72ri_retry_ssplit:
        subq.l  #1,d3
        beq.w   .c72ri_return
        bsr     bulk_wait_next_frame
        bra.w   .c72ri_ssplit

.c72ri_return:
        clr.b   split_complete
        move.l  d5,d0
        btst    #31,d0
        bne.s   .c72ri_recover
        move.l  d0,d6
        and.l   #HCINT_ERROR_MASK,d6
        beq.s   .c72ri_clean
.c72ri_recover:
        bsr     channel0_recover_timeout
.c72ri_clean:
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32
        move.l  d5,d0
        rts

.c72ri_startbad:
        clr.b   split_complete
        bsr     channel0_recover_timeout
        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32
        moveq   #0,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32
        move.l  #$80000000,d0
        rts

; Wait until HFNUM advances before retrying a NAK/NYET bulk transaction.
; This keeps retries aligned with real USB scheduling rather than spinning the
; same halted transaction immediately.
bulk_wait_next_frame:
        movem.l d0-d2/a0,-(sp)
        move.l  #HFNUM,a0
        bsr     mmio_read32
        and.l   #$0000FFFF,d0
        move.l  d0,d2
        move.l  #$00020000,d1
.bwnf_loop:
        move.l  #HFNUM,a0
        bsr     mmio_read32
        and.l   #$0000FFFF,d0
        cmp.l   d2,d0
        bne.s   .bwnf_done
        subq.l  #1,d1
        bne.s   .bwnf_loop
.bwnf_done:
        movem.l (sp)+,d0-d2/a0
        rts

channel0_wait:
        move.l  #TIMEOUT,d7
        clr.l   c0w_seen
C0WLoop:
        move.l  #HCINT0,a0
        bsr     mmio_read32
        move.l  d0,d6
        or.l    d6,c0w_seen

        move.l  d6,d0
        and.l   #HCINT_ERROR_MASK,d0
        bne.s   C0WDone

        btst    #1,d6
        bne.s   C0WDone

        subq.l  #1,d7
        bne.s   C0WLoop

        move.l  #$80000000,d6
        or.l    c0w_seen,d6
        move.l  d6,d0
        rts
C0WDone:
        move.l  c0w_seen,d0
        rts

transfer_status_ok:
        move.l  d0,d6
        btst    #31,d6
        bne.s   TSOBad
        move.l  d6,d0
        and.l   #HCINT_ERROR_MASK,d0
        bne.s   TSOBad
        btst    #0,d6
        beq.s   TSOBad
        moveq   #0,d0
        rts
TSOBad:
        moveq   #1,d0
        rts

; ================================================================
; Core / root port
; ================================================================

init_core_host:
        moveq   #0,d0
        move.l  #GINTMSK,a0
        bsr     mmio_write32

        move.l  #GAHBCFG,a0
        bsr     mmio_read32
        and.l   #$FFFFFFFE,d0
        move.l  #GAHBCFG,a0
        bsr     mmio_write32

        move.l  #GUSBCFG,a0
        bsr     mmio_read32
        and.l   #$FFAFFFFF,d0
        move.l  #GUSBCFG,a0
        bsr     mmio_write32

        move.l  #GRSTCTL_AHBIDLE,d1
        move.l  #GRSTCTL,a0
        bsr     wait_bit_set
        tst.l   d0
        bne.w   ICHFail

        move.l  #GRSTCTL,a0
        bsr     mmio_read32
        or.l    #GRSTCTL_CSFTRST,d0
        move.l  #GRSTCTL,a0
        bsr     mmio_write32

        move.l  #GRSTCTL_CSFTRST,d1
        move.l  #GRSTCTL,a0
        bsr     wait_bit_clear
        tst.l   d0
        bne.w   ICHFail

        move.l  DosBasePtr,a6
        moveq   #5,d1
        jsr     Delay(a6)

        move.l  #GUSBCFG,a0
        bsr     mmio_read32
        and.l   #$FFF7FFF7,d0
        and.l   #$FFFDFCEF,d0
        move.l  #GUSBCFG,a0
        bsr     mmio_write32

        move.l  #GAHBCFG,a0
        bsr     mmio_read32
        and.l   #$FFFFFFF8,d0
        or.l    #GAHBCFG_WAIT_AXI+GAHBCFG_DMA_EN,d0
        move.l  #GAHBCFG,a0
        bsr     mmio_write32

        moveq   #0,d0
        move.l  #USB_POWER,a0
        bsr     mmio_write32

        move.l  #HCFG,a0
        bsr     mmio_read32
        and.l   #$FFFFFFFC,d0
        move.l  #HCFG,a0
        bsr     mmio_write32

        move.l  #$00000400,d0
        move.l  #GRXFSIZ,a0
        bsr     mmio_write32

        move.l  #$04000400,d0
        move.l  #GNPTXFSIZ,a0
        bsr     mmio_write32

        move.l  #$04000800,d0
        move.l  #HPTXFSIZ,a0
        bsr     mmio_write32

        move.l  #GRSTCTL,a0
        bsr     mmio_read32
        or.l    #GRSTCTL_TXFFLSH+GRSTCTL_TXFNUM_ALL,d0
        move.l  #GRSTCTL,a0
        bsr     mmio_write32

        move.l  #GRSTCTL_TXFFLSH,d1
        move.l  #GRSTCTL,a0
        bsr     wait_bit_clear
        tst.l   d0
        bne.s   ICHFail

        move.l  #GRSTCTL,a0
        bsr     mmio_read32
        or.l    #GRSTCTL_RXFFLSH,d0
        move.l  #GRSTCTL,a0
        bsr     mmio_write32

        move.l  #GRSTCTL_RXFFLSH,d1
        move.l  #GRSTCTL,a0
        bsr     wait_bit_clear
        tst.l   d0
        bne.s   ICHFail

        moveq   #0,d0
        move.l  #HCINTMSK0,a0
        bsr     mmio_write32

        move.l  #$FFFFFFFF,d0
        move.l  #HCINT0,a0
        bsr     mmio_write32

        moveq   #0,d0
        move.l  #HCSPLT0,a0
        bsr     mmio_write32

        moveq   #0,d0
        rts
ICHFail:
        moveq   #1,d0
        rts

prepare_root_port:
        bsr     read_hprt
        move.l  d6,d0
        and.l   #HPRT_SAFE_MASK,d0
        or.l    #HPRT_PWR,d0
        bsr     write_hprt_d0

        move.l  DosBasePtr,a6
        move.l  #25,d1
        jsr     Delay(a6)

        bsr     read_hprt
        move.l  d6,d0
        btst    #12,d0
        beq.s   PRPFail
        btst    #0,d0
        beq.s   PRPFail

        move.l  DosBasePtr,a6
        moveq   #5,d1
        jsr     Delay(a6)

        bsr     read_hprt
        move.l  d6,d0
        and.l   #HPRT_SAFE_MASK,d0
        or.l    #HPRT_PWR+HPRT_RST,d0
        bsr     write_hprt_d0

        move.l  DosBasePtr,a6
        moveq   #5,d1
        jsr     Delay(a6)

        bsr     read_hprt
        move.l  d6,d0
        and.l   #HPRT_SAFE_NORST,d0
        or.l    #HPRT_PWR,d0
        bsr     write_hprt_d0

        move.l  DosBasePtr,a6
        moveq   #2,d1
        jsr     Delay(a6)

        bsr     read_hprt
        move.l  d6,d0
        btst    #0,d0
        beq.s   PRPFail
        btst    #2,d0
        beq.s   PRPFail
        btst    #8,d0
        bne.s   PRPFail

        moveq   #0,d0
        rts
PRPFail:
        moveq   #1,d0
        rts

; ================================================================
; DMA mapping
; ================================================================

dma_pre:
        move.l  a0,dma_vaddr
        move.l  d0,dma_req_len
        move.l  d0,dma_len
        move.l  d1,dma_flags

        move.l  a0,a2
        lea     dma_len,a1
        move.l  d1,d0
        move.l  SysBasePtr,a6
        move.l  a2,a0
        jsr     CachePreDMA(a6)

        move.l  d0,dma_phys
        move.l  dma_len,d2
        cmp.l   dma_req_len,d2
        bcs.s   DMAPFail

        move.l  dma_phys,d0
        and.l   #$3FFFFFFF,d0
        or.l    #$C0000000,d0
        move.l  d0,dma_bus
        moveq   #0,d0
        rts
DMAPFail:
        move.l  dma_vaddr,a0
        lea     dma_len,a1
        move.l  dma_flags,d0
        move.l  SysBasePtr,a6
        jsr     CachePostDMA(a6)
        moveq   #1,d0
        rts

dma_post:
        move.l  d0,dma_len
        lea     dma_len,a1
        move.l  d1,d0
        move.l  SysBasePtr,a6
        jsr     CachePostDMA(a6)
        rts

; ================================================================
; MMIO / waits
; ================================================================

mmio_read32:
        move.l  (a0),d0
        bsr     bswap32
        rts

mmio_write32:
        bsr     bswap32
        move.l  d0,(a0)
        rts

read_hprt:
        move.l  #HPRT0,a0
        bsr     mmio_read32
        move.l  d0,d6
        rts

write_hprt_d0:
        move.l  #HPRT0,a0
        bsr     mmio_write32
        rts

wait_bit_set:
        movem.l d2-d3/a0/a2,-(sp)
        move.l  a0,a2
        move.l  d1,d2
        move.l  #TIMEOUT,d3
WBSLoop:
        move.l  a2,a0
        bsr     mmio_read32
        and.l   d2,d0
        bne.s   WBSOK
        subq.l  #1,d3
        bne.s   WBSLoop
        moveq   #1,d0
        bra.s   WBSDone
WBSOK:
        moveq   #0,d0
WBSDone:
        movem.l (sp)+,d2-d3/a0/a2
        rts

wait_bit_clear:
        movem.l d2-d3/a0/a2,-(sp)
        move.l  a0,a2
        move.l  d1,d2
        move.l  #TIMEOUT,d3
WBCLoop:
        move.l  a2,a0
        bsr     mmio_read32
        and.l   d2,d0
        beq.s   WBCOK
        subq.l  #1,d3
        bne.s   WBCLoop
        moveq   #1,d0
        bra.s   WBCDone
WBCOK:
        moveq   #0,d0
WBCDone:
        movem.l (sp)+,d2-d3/a0/a2
        rts

; ================================================================
; Firmware mailbox
; ================================================================

build_set_power:
        move.l  prop_ptr,a2
        move.l  a2,a0
        move.l  #PROP_MSG_SIZE,d0
        bsr     store_le32

        lea     4(a2),a0
        moveq   #0,d0
        bsr     store_le32

        lea     8(a2),a0
        move.l  #TAG_SET_POWER,d0
        bsr     store_le32

        lea     12(a2),a0
        moveq   #8,d0
        bsr     store_le32

        lea     16(a2),a0
        moveq   #8,d0
        bsr     store_le32

        lea     20(a2),a0
        moveq   #DEV_USB_HCD,d0
        bsr     store_le32

        lea     24(a2),a0
        moveq   #3,d0
        bsr     store_le32

        lea     28(a2),a0
        moveq   #0,d0
        bsr     store_le32
        rts

prop_dma_pre:
        move.l  #PROP_MSG_SIZE,prop_len
        move.l  prop_ptr,a0
        lea     prop_len,a1
        moveq   #0,d0
        move.l  SysBasePtr,a6
        jsr     CachePreDMA(a6)
        move.l  d0,prop_phys
        rts

prop_dma_post:
        move.l  #PROP_MSG_SIZE,prop_len
        move.l  prop_ptr,a0
        lea     prop_len,a1
        moveq   #0,d0
        move.l  SysBasePtr,a6
        jsr     CachePostDMA(a6)
        rts

mailbox_property:
        move.l  prop_phys,d5
        and.l   #$FFFFFFF0,d5
        or.l    #MBOX_PROPERTY,d5

        move.l  #TIMEOUT,d7
MBTXWait:
        move.l  MBOX_STATUS,d0
        bsr     bswap32
        and.l   #MBOX_TX_FULL,d0
        beq.s   MBTXReady
        subq.l  #1,d7
        bne.s   MBTXWait
        moveq   #1,d0
        rts
MBTXReady:
        move.l  d5,d0
        bsr     bswap32
        move.l  d0,MBOX_WRITE

        move.l  #TIMEOUT,d7
MBRXWait:
        move.l  MBOX_STATUS,d0
        bsr     bswap32
        and.l   #MBOX_RX_EMPTY,d0
        bne.s   MBRXNext

        move.l  MBOX_READ,d0
        bsr     bswap32
        move.l  d0,d6

        move.l  d6,d0
        and.l   #$0000000F,d0
        cmp.l   #MBOX_PROPERTY,d0
        bne.s   MBRXNext

        move.l  d6,d0
        and.l   #$FFFFFFF0,d0
        move.l  d5,d1
        and.l   #$FFFFFFF0,d1
        cmp.l   d1,d0
        beq.s   MBOK
MBRXNext:
        subq.l  #1,d7
        bne.s   MBRXWait
        moveq   #1,d0
        rts
MBOK:
        moveq   #0,d0
        rts

; ================================================================
; Endian
; ================================================================

bswap32:
        ror.w   #8,d0
        swap    d0
        ror.w   #8,d0
        rts

store_le32:
        bsr     bswap32
        move.l  d0,(a0)
        rts

; ================================================================
; Resident strings
; ================================================================

DevName:
        dc.b    "berrypi3ap.device",0
        even
DosName:
        dc.b    "dos.library",0
        even
IdString:
        dc.b    "berrypi3ap.device 74.0 - Build MichiB210 + Ki+Ki - USB74 fix: release IO gate on active-HID NAK (multi-device-on-hub hang)",13,10,0
        even
VendorString:
        dc.b    "Broadcom",0
        even
ProductString:
        dc.b    "BCM2837 DWC2",0
        even
DescriptionString:
        dc.b    "Raspberry Pi 3 DWC2 USB 2.0 Host Controller Driver",0
        even
LicenseString:
        dc.b    "Development build",0
        even

; USB57 virtual USB 2.0 one-port root hub descriptors.
RootHubDeviceDesc:
        dc.b    $12,$01,$00,$02,$09,$00,$01,$40,$00,$00,$00,$00,$00,$01,$00,$00,$00,$01
        even
RootHubConfigTree:
        dc.b    $09,$02,$19,$00,$01,$01,$00,$C0,$00
        dc.b    $09,$04,$00,$00,$01,$09,$00,$01,$00
        dc.b    $07,$05,$81,$03,$01,$00,$0C
        even
RootHubHubDesc:
        dc.b    $09,$29,$01,$09,$00,$01,$00,$00,$FF
        even

EndResident:
        nop

        section data,data

SysBasePtr:
        dc.l    0
DosBasePtr:
        dc.l    0
CurrentIOReq:
        dc.l    0

driver_state:
        dc.w    UHSF_OPERATIONAL
io_gate_busy:
        dc.b    0
        even
hw_ready:
        dc.b    0
current_addr:
        dc.b    0
root_hub_addr:
        dc.b    0
root_hub_config:
        dc.b    0
root_last_conn:
        dc.b    0
        even
root_change_bits:
        dc.w    0
ctrl_error_stage:
        dc.b    0
        even

prop_alloc:
        dc.l    0
prop_ptr:
        dc.l    0
prop_len:
        dc.l    0
prop_phys:
        dc.l    0

dma_alloc:
        dc.l    0
dma_base:
        dc.l    0
setup_ptr:
        dc.l    0
data_ptr:
        dc.l    0
status_ptr:
        dc.l    0

dma_vaddr:
        dc.l    0
dma_req_len:
        dc.l    0
dma_len:
        dc.l    0
dma_flags:
        dc.l    0
dma_phys:
        dc.l    0
dma_bus:
        dc.l    0

req_value:
        dc.w    0
req_index:
        dc.w    0
req_length:
        dc.w    0
req_bm:
        dc.b    0
req_b:
        dc.b    0
        even
ctrl_mps:
        dc.w    64
ctrl_flags:
        dc.w    0
ctrl_split_hub:
        dc.w    0
ctrl_split_port:
        dc.w    0
ctrl_split_effective:
        dc.w    0
ctrl_actual:
        dc.l    0
ctrl_data_offset:
        dc.l    0
ctrl_chunk_len:
        dc.l    0
ctrl_split_dma_len:
        dc.l    0
ctrl_data_pid:
        dc.l    0
ctrl_final_hctsiz:
        dc.l    0
split_complete:
        dc.b    0
        even

intr_endpoint:
        dc.w    0
intr_mps:
        dc.w    0
intr_interval:
        dc.w    0
intr_flags:
        dc.w    0
intr_split_hub:
        dc.w    0
intr_split_port:
        dc.w    0
intr_split_effective:
        dc.w    0
        even
intr_length:
        dc.l    0
intr_buffer:
        dc.l    0
intr_dma_buffer:
        dc.l    0
intr_dma_length:
        dc.l    0
intr_dma_flags:
        dc.l    0
intr_hctsiz:
        dc.l    0
intr_final_hctsiz:
        dc.l    0
intr_actual:
        dc.l    0
intr_hcint:
        dc.l    0
intr_retry_left:
        dc.l    0
intr_retry_count:
        dc.l    0
intr_recovered_timeouts:
        dc.l    0
intr_in_toggle:
        dc.b    0                       ; retained for V57 layout/debug compatibility
        even
intr_toggle_bits:
        dcb.b   256,0                   ; DATA1-next bit per address/endpoint
        even
intr_seen_bits:
        dcb.b   256,0                   ; USB64/66: pipe has delivered real HID data
        even

bulk_endpoint:
        dc.w    0
bulk_direction:
        dc.w    0
bulk_mps:
        dc.w    0
        even
bulk_length:
        dc.l    0
bulk_buffer:
        dc.l    0
bulk_dma_buffer:
        dc.l    0
bulk_dma_length:
        dc.l    0
bulk_dma_flags:
        dc.l    0
bulk_hctsiz:
        dc.l    0
bulk_pktcnt:
        dc.l    0
bulk_final_hctsiz:
        dc.l    0
bulk_actual:
        dc.l    0
bulk_hcint:
        dc.l    0
bulk_retry_left:
        dc.l    0
bulk_retry_count:
        dc.l    0
bulk_out_toggle:
        dc.b    0
bulk_in_toggle:
        dc.b    0
        even

xfer_hctsiz:
        dc.l    0
xfer_dir:
        dc.l    0
xfer_bus:
        dc.l    0
c0w_seen:
        dc.l    0

ctrl_setup_status:
        dc.l    0
ctrl_data_status:
        dc.l    0
ctrl_status_status:
        dc.l    0
ctrl_error_hcint:
        dc.l    0
