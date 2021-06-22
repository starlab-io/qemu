

static const char * igb_vf_reg2name(hwaddr offset)
{
    switch (offset) {
    case 0x00000:
        return "VTCTRL";
    case 0x00008:
        return "VTSTATUS";
    case 0x01048:
        return "VTFRTIMER";
    case 0x01520:
        return "VTEICS";
    case 0x01524:
        return "VTEIMS";
    case 0x01528:
        return "VTEIMC";
    case 0x0152C:
        return "VTEIAC";
    case 0x01530:
        return "VTEIAM";
    case 0x01580:
        return "VTEICR";
    case 0x01700:
        return "VTIVAR0";
    case 0x01740:
        return "VTIVAR_MISC";
    case 0x00f04:
        return "VTPBACL";
    case 0x00f0c:
        return "VTPSRTYPE";
    case 0x00c40:
        return "VTMAILBOX";
    case 0x00800:
        return "VMBMEM";

    case 0x01680:
        return "VTEITR0";
    case 0x01684:
        return "VTEITR1";
    case 0x01688:
        return "VTEITR2";

    case 0x02800:
        return "VTRDBAL0";
    case 0x02804:
        return "VTRDBAH0";
    case 0x02808:
        return "VTRDLEN0";
    case 0x0280C:
        return "VTSRRCTL0";
    case 0x02810:
        return "VTRDH0";
    case 0x02818:
        return "VTRDT0";
    case 0x02828:
        return "VTRXDCTL0";
    case 0x03800:
        return "VTTDBAL0";
    case 0x03804:
        return "VTTDBAH0";
    case 0x03808:
        return "VTTDLEN0";
    case 0x03810:
        return "VTTDH0";
    case 0x03814:
        return "VTTXCTL0";
    case 0x03818:
        return "VTTDT0";
    case 0x03828:
        return "VTTXDCTL0";



    case 0x02900:
        return "VTRDBAL1";
    case 0x02904:
        return "VTRDBAH1";
    case 0x02908:
        return "VTRDLEN1";
    case 0x0290C:
        return "VTSRRCTL1";
    case 0x02910:
        return "VTRDH1";
    case 0x02918:
        return "VTRDT1";
    case 0x02928:
        return "VTRXDCTL1";
    case 0x03900:
        return "VTTDBAL1";
    case 0x03904:
        return "VTTDBAH1";
    case 0x03908:
        return "VTTDLEN1";
    case 0x03910:
        return "VTTDH1";
    case 0x03914:
        return "VTTXCTL1";
    case 0x03918:
        return "VTTDT1";
    case 0x03928:
        return "VTTXDCTL1";

    case 0x0F10 :
        return "VFGPRC (stat)"; //Good Packets Received Count
    case 0x0F14 :
        return "VFGPTC (stat)"; //Good Packets Transmitted Count
    case 0x0F18 :
        return "VFGORC (stat)"; //Good Octets Received Count
    case 0x0F34 :
        return "VFGOTC (stat)"; //Good Octets Transmitted Count
    case 0x0F3C :
        return "VFMPRC (stat)"; //Multicast Packets Received Count
    case 0x0F40 :
        return "VFGPRLBC (stat)"; //Good RX Packets loopback Count
    case 0x0F44 :
        return "VFGPTLBC (stat)"; //Good TX packets loopback Count
    case 0x0F48 :
        return "VFGORLBC (stat)"; //Good RX Octets loopback Count
    case 0x0F50 :
        return "VFGOTLBC (stat)"; //Good TX Octets loopback Count

    default:
        return "???";
    }
}
