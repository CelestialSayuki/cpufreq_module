#include <errno.h>
#include <pongo.h>

#include "cpufreq_private.h"
#include "utils.h"

void s8000_hw_init(uint64_t cluster_base);
void t8015_hw_init(uint64_t cluster_base);

uint64_t get_frequency_for_state(int state)
{
    uint64_t freq = data.hw_config->get_frequency_for_state(data.hw_config->cluster_base, state);

    if (!freq)
        freq = BASE_CLOCK;

    return freq;
}

static uint64_t parse_u64(const char *str) {
    uint64_t res = 0;
    int base = 10;
    if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X')) {
        base = 16;
        str += 2;
    }
    while (*str) {
        char c = *str;
        if (c >= '0' && c <= '9') res = res * base + (c - '0');
        else if (c >= 'a' && c <= 'f') res = res * base + (c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') res = res * base + (c - 'A' + 10);
        else break;
        str++;
    }
    return res;
}

int set_state(int state)
{
    return data.hw_config->set_state(data.hw_config->cluster_base, state);
}

uint64_t get_state(void)
{
    return data.hw_config->get_state(data.hw_config->cluster_base);
}

uint64_t get_target_state(void)
{
    return data.hw_config->get_target_state(data.hw_config->cluster_base);
}

uint64_t get_core_type_for_state(int state)
{
    if (data.hw_config->get_core_type_for_state)
        return data.hw_config->get_core_type_for_state(data.hw_config->cluster_base, state);
    else
        return CORE_TYPE_UNKNOWN;
}

void cpufreq_show(const char *cmd, char *args)
{
    uint32_t state = get_state();
    uint32_t target = get_target_state();
    uint64_t freq = get_frequency_for_state(state);

    printf("Current state           : %u\n", state);
    printf("Current target state    : %u\n", target);
    printf("Current CPU frequency   : %llu Hz\n", freq);

    if (data.hw_config->get_core_type_for_state)
        printf("Current CPU type        : %s\n", core_type_array[get_core_type_for_state(state)]);

    if (data.hw_config->get_vcore)
        printf("Vcore                   : %u mV\n",
               data.hw_config->get_vcore(data.hw_config->cluster_base, state));

    if ((state > data.max_nonboost_pstate) && (state <= data.max_configured_pstate))
        printf("Boost state\n");
}

void cpufreq_dump(const char *cmd, char *args)
{
    printf("%-4s | %-18s | %-18s | %-15s | %-9s | %-8s | %s\n",
           "ID", "Register Addr", "Raw Value", "Freq (Hz)", "Volt(mV)", "Type", "Note");
    printf("-----|--------------------|--------------------|-----------------|-----------|----------|------\n");

    for (uint32_t i = 1; i <= data.max_configured_pstate; i++) {
        uint64_t addr = 0;
        uint64_t base_addr = data.hw_config->cluster_base;

        switch (socnum) {
            case 0x8960:
            case 0x7000:
            case 0x7001:
                addr = base_addr + CLUSTER_PSINFO1_S5L8960X(i);
                break;
            case 0x8000:
            case 0x8001:
            case 0x8003:
                addr = base_addr + CLUSTER_PSINFO1_S5L8960X(i);
                break;
            case 0x8010:
            case 0x8011:
            case 0x8012:
                addr = base_addr + CLUSTER_PSINFO1_T8010(i);
                break;
            case 0x8015:
                if (i >= 8 && data.hw_config->pcluster_base) {
                    addr = data.hw_config->pcluster_base + CLUSTER_PSINFO1_T8015(i);
                } else {
                    addr = base_addr + CLUSTER_PSINFO1_T8015(i);
                }
                break;
            default:
                addr = 0;
        }

        uint64_t raw_val = (addr != 0) ? read64(addr) : 0;

        uint64_t freq = get_frequency_for_state(i);
        
        uint32_t volt = 0;
        if (data.hw_config->get_vcore)
            volt = data.hw_config->get_vcore(base_addr, i);

        const char *core_type = "-";
        if (data.hw_config->get_core_type_for_state)
            core_type = core_type_array[get_core_type_for_state(i)];

        const char *note = "";
        if ((i > data.max_nonboost_pstate) && (i <= data.max_configured_pstate))
            note = "Boost";

        if (addr != 0) {
            printf("%-4u | 0x%016llx | 0x%016llx | %-15llu | %-9u | %-8s | %s\n",
                   i, addr, raw_val, freq, volt, core_type, note);
        } else {
            printf("%-4u | %-18s | %-18s | %-15llu | %-9u | %-8s | %s\n",
                   i, "N/A", "N/A", freq, volt, core_type, note);
        }
    }
}

void cpufreq_set(const char *cmd, char *args)
{
    if (!*args) {
        printf("usage: cpufreq set [state]\n");
        printf("\nState is a number from 0 to a device-dependent maximum value\n");
        printf("Run `cpufreq dump` to get more information.\n");
        return;
    }

    uint64_t state = parse_u64(args);
    
    if (!state && *args != '0') {
    }

    if ((uint32_t)state > data.max_configured_pstate) {
        printf("Invalid pstate specified. Max pstate is %u\n", data.max_configured_pstate);
        return;
    }

    set_state(state);
}

void cpufreq_dt(const char *cmd, char *args)
{
    struct dt_node *pmgr_node = dt_find(gDeviceTree, "/arm-io/pmgr");
    if (!pmgr_node) {
        printf("cpufreq: /arm-io/pmgr not found!\n");
        return;
    }
    int show_magic = 0;
    switch (socnum) {
        case 0x8960: case 0x7000: case 0x7001:
            show_magic = 1;
            break;
        default:
            show_magic = 0;
            break;
    }
    const char *prop_names[] = {"voltage-states1", "voltage-states5", NULL};
    uint64_t base_addr = data.hw_config->cluster_base;
    for (int i = 0; prop_names[i]; i++) {
        char *name = (char *)prop_names[i];
        uint32_t len = 0;
        void *prop_ptr = dt_prop(pmgr_node, name, &len);
        if (!prop_ptr || len == 0) continue;
        printf("Found DT Property: '%s' (Length: %u bytes)\n", name, len);
        if (show_magic) {
            printf("%-5s | %-10s | %-8s | %-11s | %-10s | %s\n",
                   "Idx", "Raw Freq", "Volt(mV)", "Decoded MHz", "Magic", "Note");
            printf("------|------------|----------|-------------|------------|------\n");
        } else {
            printf("%-5s | %-10s | %-8s | %-11s | %s\n",
                   "Idx", "Raw Freq", "Volt(mV)", "Decoded MHz", "Note");
            printf("------|------------|----------|-------------|------\n");
        }
        uint32_t *states = (uint32_t *)prop_ptr;
        uint32_t count = len / 8;
        if (count > 32) count = 32;
        for (uint32_t j = 0; j < count; j++) {
            uint32_t raw_freq = states[j * 2];
            uint32_t volt = states[j * 2 + 1];
            uint32_t decoded_mhz = (raw_freq > 0) ? (uint32_t)(65536000000000ULL / raw_freq) / 1000000 : 0;
            printf("%-5u | %-10u | %-8u | %-11u | ", j, raw_freq, volt, decoded_mhz);
            if (show_magic) {
                uint32_t magic_val = 0;
                int valid_magic = 0;
                if (raw_freq > 0) {
                    int hw_state = j + 2;
                    if (hw_state <= data.max_configured_pstate) {
                        uint64_t reg_addr = base_addr + CLUSTER_PSINFO1_S5L8960X(hw_state);
                        uint64_t reg = read64(reg_addr);
                        magic_val = (uint32_t)((reg >> 16) & 0xFF);
                        valid_magic = 1;
                    }
                }
                if (valid_magic) {
                    printf("0x%-8x | ", magic_val);
                } else {
                    printf("%-10s | ", "-");
                }
            }
            if (raw_freq == 0) {
                printf("(Empty)");
            }
            printf("\n");
        }
        printf("\n");
    }
}

void cpufreq_edit(const char *cmd, char *args)
{
    char *str_state = args;
    char *str_freq = command_tokenize(str_state, 0x1ff);
    char *str_volt = str_freq ? command_tokenize(str_freq, 0x1ff) : NULL;
    char *str_magic = str_volt ? command_tokenize(str_volt, 0x1ff) : NULL;
    if (!str_state || !*str_state || !str_freq || !*str_freq || !str_volt || !*str_volt) {
        printf("usage: cpufreq edit <state> <freq_mhz> <voltage_mv> [magic_hex]\n");
        return;
    }
    uint32_t state = (uint32_t)parse_u64(str_state);
    uint32_t input_freq = (uint32_t)parse_u64(str_freq);
    uint32_t volt_mv = (uint32_t)parse_u64(str_volt);
    uint32_t real_freq_mhz = 0;
    uint64_t real_volt_uv = 0;
    if (state > data.max_configured_pstate) {
        printf("cpufreq: Invalid state %u (Max: %u)\n", state, data.max_configured_pstate);
        return;
    }
    uint64_t val_mul = 0;
    uint64_t val_vcore = 0;
    uint64_t reg_addr = 0;
    uint64_t reg_val = 0;
    uint64_t base_addr = data.hw_config->cluster_base;
    switch (socnum) {
        case 0x7000: case 0x7001: case 0x8960:
            uint64_t div2_val = 0;
            if (input_freq % 24 != 0) {
                div2_val = 1;
                val_mul = input_freq / 12;
            } else {
                div2_val = 0;
                val_mul = input_freq / 24;
            }
            real_freq_mhz = (uint32_t)((val_mul * 24) / (div2_val + 1));
            if (volt_mv < 600) volt_mv = 600;
            val_vcore = ((uint64_t)(volt_mv - 600) * 1000) / 3125;
            real_volt_uv = 600000 + (val_vcore * 3125);
            reg_addr = base_addr + CLUSTER_PSINFO1_S5L8960X(state);
            uint64_t old_val = read64(reg_addr);
            uint64_t magic_bits;
            if (str_magic && *str_magic) {
                magic_bits = parse_u64(str_magic);
            } else {
                magic_bits = (old_val >> 16) & 0xFF;
                if (magic_bits == 0) magic_bits = 0x38;
            }
            reg_val = (magic_bits << 16) |
                      FIELD_PREP(CLUSTER_PSINFO1_VCORE_S8000, val_vcore & 0xFF) |
                      FIELD_PREP(MUL_S5L8960X, val_mul & 0x1FF) |
                      FIELD_PREP(DIV1_S5L8960X, 1) |
                      FIELD_PREP(DIV2_S5L8960X, div2_val);
            printf("cpufreq: Edit -> Freq: %u MHz (Reg: %u MHz) | Volt: %u mV (Real: %u.%03u mV) | Magic: 0x%llx\n",
                    input_freq, real_freq_mhz,
                    volt_mv, (uint32_t)(real_volt_uv / 1000), (uint32_t)(real_volt_uv % 1000),
                    magic_bits);
            break;
        case 0x8000: case 0x8001: case 0x8003:
            val_mul = input_freq / 12;
            real_freq_mhz = val_mul * 12;
            if (volt_mv < 450) volt_mv = 450;
            val_vcore = ((uint64_t)(volt_mv - 450) * 1000) / 3125;
            real_volt_uv = 450000 + (val_vcore * 3125);
            if (state < 8) reg_addr = base_addr + CLUSTER_PSINFO1_OFF_S5L8960X + (state * 8);
            else reg_addr = base_addr + CLUSTER_PSINFO1_EXT_OFF_S8000 + ((state - 8) * 8);
            reg_val = read64(reg_addr);
            reg_val &= ~(CLUSTER_PSINFO1_VCORE_S8000 | MUL_S8000 | DIV_S8000 | MOD_S8000);
            reg_val |= FIELD_PREP(CLUSTER_PSINFO1_VCORE_S8000, val_vcore & 0xFF) |
                       FIELD_PREP(MUL_S8000, val_mul & 0x1FF);
            printf("cpufreq: Edit -> Freq: %u MHz (Reg: %u MHz) | Volt: %u mV (Real: %u.%03u mV)\n",
                   input_freq, real_freq_mhz,
                   volt_mv, (uint32_t)(real_volt_uv / 1000), (uint32_t)(real_volt_uv % 1000));
            break;
        case 0x8010: case 0x8011: case 0x8012:
            val_mul = input_freq / 12;
            real_freq_mhz = val_mul * 12;
            if (volt_mv < 500) volt_mv = 500;
            val_vcore = ((uint64_t)(volt_mv - 500) * 1000 + 500) / 3125;
            real_volt_uv = 500000 + ((val_vcore * 3125) - 500);
            reg_addr = base_addr + CLUSTER_PSINFO1_T8010(state);
            reg_val = read64(reg_addr);
            reg_val &= ~(CLUSTER_PSINFO1_VCORE_S8000 | MUL_S8000 | DIV_S8000 | MOD_S8000);
            reg_val |= FIELD_PREP(CLUSTER_PSINFO1_VCORE_S8000, val_vcore & 0xFF) |
                       FIELD_PREP(MUL_S8000, val_mul & 0x1FF);
            printf("cpufreq: Edit -> Freq: %u MHz (Reg: %u MHz) | Volt: %u mV (Real: %u.%03u mV)\n",
                   input_freq, real_freq_mhz,
                   volt_mv, (uint32_t)(real_volt_uv / 1000), (uint32_t)(real_volt_uv % 1000));
            break;
        case 0x8015:
            val_mul = input_freq / 12;
            real_freq_mhz = val_mul * 12;
            if (volt_mv < 375) volt_mv = 375;
            val_vcore = ((uint64_t)(volt_mv - 375) * 1000 + 500) / 3125;
            real_volt_uv = 375000 + ((val_vcore * 3125) - 500);
            if (state >= 8 && data.hw_config->pcluster_base) base_addr = data.hw_config->pcluster_base;
            reg_addr = base_addr + CLUSTER_PSINFO1_T8015(state);
            reg_val = read64(reg_addr);
            reg_val &= ~(CLUSTER_PSINFO1_VCORE_S8000 | MUL_S8000 | DIV_S8000 | MOD_S8000);
            reg_val |= FIELD_PREP(CLUSTER_PSINFO1_VCORE_S8000, val_vcore & 0xFF) |
                       FIELD_PREP(MUL_S8000, val_mul & 0x1FF);
            printf("cpufreq: Edit -> Freq: %u MHz (Reg: %u MHz) | Volt: %u mV (Real: %u.%03u mV)\n",
                   input_freq, real_freq_mhz,
                   volt_mv, (uint32_t)(real_volt_uv / 1000), (uint32_t)(real_volt_uv % 1000));
            break;
        default:
            printf("cpufreq: Edit not supported on this SoC (0x%x)\n", socnum);
            return;
    }
    write64(reg_addr, reg_val);
    if (real_freq_mhz > 0) {
        uint64_t dt_freq_hz = (uint64_t)real_freq_mhz * 1000000ULL;
        uint32_t dt_freq_val = (uint32_t)(65536000000000ULL / dt_freq_hz);
        char *prop_name = "voltage-states1";
        if (socnum == 0x8015 && state >= 8) prop_name = "voltage-states5";
        struct dt_node *pmgr_node = dt_find(gDeviceTree, "/arm-io/pmgr");
        if (pmgr_node) {
            uint32_t prop_len = 0;
            void *prop_ptr = dt_prop(pmgr_node, prop_name, &prop_len);
            int dt_index = (int)state;
            if (socnum == 0x8960 || socnum == 0x7000 || socnum == 0x7001) {
                dt_index = state - 2;
            }
            if (prop_ptr && dt_index >= 0 && prop_len >= (dt_index * 8 + 8)) {
                uint32_t *states_array = (uint32_t *)prop_ptr;
                uint32_t rounded_mv = (uint32_t)((real_volt_uv + 500) / 1000);
                states_array[dt_index * 2] = dt_freq_val;
                states_array[dt_index * 2 + 1] = rounded_mv;
                printf("cpufreq: DT Sync -> %s[%d] Updated (Freq: %u MHz, Volt: %u mV)\n",
                       prop_name, dt_index, real_freq_mhz, rounded_mv);
            }
        }
    }
}

void cpufreq_unlock(const char *cmd, char *args)
{
    if (data.max_configured_pstate == data.max_nonboost_pstate)
        return;

    for (uint32_t i = (data.max_nonboost_pstate + 1); i <= data.max_configured_pstate; i++) {
        uint64_t addr = 0;

        switch (socnum) {
            case 0x8000:
            case 0x8001:
            case 0x8003:
                addr = data.hw_config->cluster_base + CLUSTER_PSINFO2_S8000(i);
                break;
            case 0x8010:
            case 0x8011:
            case 0x8012:
                addr = data.hw_config->cluster_base + CLUSTER_PSINFO2_T8010(i);
                ;
                break;
            case 0x8015:
                addr = data.hw_config->cluster_base + CLUSTER_PSINFO2_T8015(i);
                break;
        }

        if (!addr) {
            printf("cpufreq: Don't know how to unlock boost state on this SoC\n");
            return;
        }

        mask64(addr, CLUSTER_PSINFO2_MAX_LOAD, FIELD_PREP(CLUSTER_PSINFO2_MAX_LOAD, 15));
    }
}

void cpufreq_magic(const char *cmd, char *args)
{
    if (data.hw_config->apply_magic)
        set_state(data.hw_config->apply_magic(data.hw_config));
    else
        printf("cpufreq: No magic available for this SoC\n");
}

void cpufreq_bench(const char *cmd, char *args)
{
    uint64_t hz = bench();

    printf("CPU frequency: %lld Hz\n", hz);
}

void cpufreq_control(const char *cmd, char *args)
{
    int enable = -1;
    if (args && !strcmp(args, "on")) enable = 1;
    else if (args && !strcmp(args, "off")) enable = 0;
    if (enable == -1) {
        printf("usage: cpufreq control <on/off>\n");
        return;
    }
    uint64_t base = data.hw_config->cluster_base;
    switch (socnum) {
        case 0x8960:
        case 0x7000:
        case 0x7001:
            if (data.hw_config->voltage_ctl) {
                write32(data.hw_config->voltage_ctl, enable ? 1 : 0);
                printf("cpufreq: Voltage Control %s (Reg: 0x%llx)\n",
                       enable ? "ENABLED" : "DISABLED", data.hw_config->voltage_ctl);
            } else {
                printf("cpufreq: Voltage control register undefined for this SoC.\n");
            }
            break;
        case 0x8015:
            if (enable) {
                t8015_hw_init(base);
                printf("cpufreq: DVFS & PLL ENABLED (APSC_DIS Cleared, PLL_EN Set)\n");
            } else {
                set64(base + CLUSTER_PSTATE_CMD, CLUSTER_PSTATE_APSC_DIS);
                clear64(base + CLUSTER_PSTATE_CMD, CLUSTER_PSTATE_PLL_EN);
                printf("cpufreq: DVFS & PLL DISABLED (APSC_DIS Set, PLL_EN Cleared)\n");
            }
            break;
        case 0x8000: case 0x8001: case 0x8003:
        case 0x8010: case 0x8011: case 0x8012:
            if (enable) {
                s8000_hw_init(base);
                printf("cpufreq: DVFS ENABLED (APSC_DIS Cleared)\n");
            } else {
                set64(base + CLUSTER_PSTATE_CMD, CLUSTER_PSTATE_APSC_DIS);
                printf("cpufreq: DVFS DISABLED (APSC_DIS Set)\n");
            }
            break;
        default:
            printf("cpufreq: Control not implemented for SoC 0x%x\n", socnum);
            break;
    }
}

#define CPUFREQ_COMMAND(_name, _desc, _cb)                                                         \
    {                                                                                              \
        .name = _name, .desc = _desc, .cb = _cb                                                    \
    }
void cpufreq_help(const char *cmd, char *args);

static struct cpufreq_command command_table[] = {
    CPUFREQ_COMMAND("help", "Show usage", cpufreq_help),
    CPUFREQ_COMMAND("dump", "Dump available CPU states", cpufreq_dump),
    CPUFREQ_COMMAND("set", "Set CPU state", cpufreq_set),
    CPUFREQ_COMMAND("edit", "Edit state (usage: edit <state> <freq_mhz> <mv>)", cpufreq_edit),
    CPUFREQ_COMMAND("dt", "Dump DeviceTree voltage states (Software)", cpufreq_dt),
    CPUFREQ_COMMAND("show", "Get current CPU information", cpufreq_show),
    CPUFREQ_COMMAND("bench", "Measure CPU frequency", cpufreq_bench),
    CPUFREQ_COMMAND("control", "Enable/Disable hardware control (usage: control <on/off>)", cpufreq_control),
    CPUFREQ_COMMAND("unlock", "Unlock boost states (Device may become unstable)", cpufreq_unlock),
    CPUFREQ_COMMAND("magic", "Apply magic (DANGEROUS)", cpufreq_magic),
};

void cpufreq_help(const char *cmd, char *args)
{
    printf("cpufreq usage: cpufreq [subcommand] <subcommand options>\nsubcommands:\n");
    for (int i = 0; i < sizeof(command_table) / sizeof(struct cpufreq_command); i++) {
        if (command_table[i].name) {
            printf("%16s | %s\n", command_table[i].name,
                   command_table[i].desc ? command_table[i].desc : "no description");
        }
    }
}

void cpufreq_cmd(const char *cmd, char *args)
{
    char *arguments = command_tokenize(args, 0x1ff - (args - cmd));
    struct cpufreq_command *fallback_cmd = NULL;
    if (arguments) {
        for (int i = 0; i < sizeof(command_table) / sizeof(struct cpufreq_command); i++) {
            if (command_table[i].name && !strcmp("help", command_table[i].name)) {
                fallback_cmd = &command_table[i];
            }
            if (command_table[i].name && !strcmp(args, command_table[i].name)) {
                command_table[i].cb(args, arguments);
                return;
            }
        }
        if (*args)
            printf("cpufreq: invalid command %s\n", args);
        if (fallback_cmd)
            return fallback_cmd->cb(cmd, arguments);
    }
}
