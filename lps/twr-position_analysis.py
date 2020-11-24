import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum

import lps
import lps_stats

# Experiment data
N_ANCHORS = 2
       
# index file
DATA_DIR = 'CZ-20200807'
cwd = os.path.dirname(__file__)
data_path = os.path.join(cwd,'../data/', DATA_DIR)

# data bin structure
MOUNTING = ['H', 'V']
TX_MODES = ['std', 'max']
RANGES = ['2m','7m']
ENTRIES = ['dist','dist_error','time']
TAG_ANGLES = ['0','45','90','135','180']

# analysis parameters
TIMEOUT_THRESHOLD = 0.15    #[s]
OUTLIER_THRESHOLD = 0.2     #[m]

def data_bin_init():
    db = dict()
    for mount in MOUNTING:
        db[mount] = dict()
        for tx_mode in TX_MODES:
            db[mount][tx_mode] = dict()
            for d_true in RANGES:
                db[mount][tx_mode][d_true] = dict()
                for entry in ENTRIES:
                    db[mount][tx_mode][d_true][entry] = dict()
                    for angle in TAG_ANGLES:
                        db[mount][tx_mode][d_true][entry][angle] = [[] for _ in range(N_ANCHORS)]
    return db


# Filter logs based on properties
def log_is_good(test_params):
    # only retain twr ranging logs with V0 anchor data over 2 or 7m     
    if test_params.anchor_orientation != 'A-V0':
        return False
    elif not (test_params.nominal_range == '2m' 
                or test_params.nominal_range == '7m'):
        return False
    elif not test_params.test_mode == 'twr-ranging':
        return False
    else:
        return True

def load_data(data_dir, data_bin):
    for log_name in os.listdir(data_dir):
        # avoid lock files
        if log_name.startswith('.'):
            continue
        else:
            log_path = os.path.join(data_dir, log_name)
            params = lps.params_from_logname(log_path)

            if not log_is_good(params):
                continue
            else:
                mnt = params.tag_orientation[2]
                ang = params.tag_orientation[3:]
                tx = params.tx_mode
                d = params.nominal_range
                
                with open(log_path, 'r') as log_file:
                    for line in log_file:
                        # skip header
                        if 'Time' in line:
                            continue
                        else:
                            items = line.split(',')
                            t = float(items[0])
                            idx = int(items[1])
                            dist = float(items[2])
                            d_err = dist - int(d[:-1])

                            data_bin[mnt][tx][d]['time'][ang][idx].append(t)
                            data_bin[mnt][tx][d]['dist'][ang][idx].append(dist)
                            data_bin[mnt][tx][d]['dist_error'][ang][idx].append(d_err)


def stat_ranging_boxplots(data_bin):
    for mnt in MOUNTING:
        fig, axs = plt.subplots(2, 2, constrained_layout=True)
        title_str = ('Vertical' if mnt=='V' else 'Horizontal') + ' Tag'
        fig.suptitle(title_str)

        for i in range(len(RANGES)):
            for j in range(len(TX_MODES)):
                for k in range(len(TAG_ANGLES)):
                    to_plot = data_bin[mnt][TX_MODES[j]][RANGES[i]]['dist_error'][TAG_ANGLES[k]]
                    pos = [1+3*k, 2+3*k]
                    bp = axs[i][j].boxplot(to_plot, positions=pos, widths=0.6)
                    boxplot_set_colors(bp)
                
                axs[i][j].set_xticks([1.5+3*k for k in range(len(TAG_ANGLES))])
                axs[i][j].set_xticklabels(TAG_ANGLES)
                axs[i][j].set_ylabel('Measured Distance [m]')
                axs[i][j].set_title('{} {}'.format(RANGES[i], TX_MODES[j]))
                axs[i][j].set_ylim(-1,1)
                axs[i][j].grid()
    plt.show()

    
def boxplot_set_colors(bp):
    plt.setp(bp['boxes'][0], color='blue')
    plt.setp(bp['caps'][0], color='blue')
    plt.setp(bp['caps'][1], color='blue')
    plt.setp(bp['whiskers'][0], color='blue')
    plt.setp(bp['whiskers'][1], color='blue')
    plt.setp(bp['fliers'][0], markeredgecolor='blue')
    plt.setp(bp['fliers'][0], marker = 'x')
    plt.setp(bp['medians'][0], color='blue')

    plt.setp(bp['boxes'][1], color='red')
    plt.setp(bp['caps'][2], color='red')
    plt.setp(bp['caps'][3], color='red')
    plt.setp(bp['whiskers'][2], color='red')
    plt.setp(bp['whiskers'][3], color='red')
    plt.setp(bp['fliers'][1], markeredgecolor='red')
    plt.setp(bp['fliers'][1], marker = 'x')
    plt.setp(bp['medians'][1], color='red')

def data_get_stats(data_bin):
    dt_min = np.amin(data_bin['dt'], axis=0)
        
    outlier_count = [0 for _ in range(N_ANCHORS)]
    for i in range(N_ANCHORS):
        for dt in data_bin['time'][i]:
            if dt > 5*dt_min:
                outlier_count[i] += 1
            else:
                continue
            
def stats_uptime_print(data):
    print('********************************')
    print('*  Connectivity Stats: Uptime  *')
    print('********************************\n')
    for mnt,mnt_data in data.items():
        for tx_mode, tx_data in mnt_data.items():
            print('Tag Orientation: {}, TX-Power: {}\n'.format(mnt, tx_mode))
            stats_uptime_block(tx_data)

def stats_uptime_block(data):
    uptime = dict()
    for d_true, d_data in data.items():
        uptime[d_true] = [[] for _ in range(N_ANCHORS)]
        for angle, ang_data in d_data['time'].items():
            for idx in range(N_ANCHORS):
                timestamps = ang_data[idx]
                t0 = min(timestamps)
                t_prev = 0
                timeout_acc = 0
                for t in timestamps:
                    dt = t-t_prev
                    t_prev = t
                    if dt>TIMEOUT_THRESHOLD:
                        timeout_acc +=dt
                    else:
                        continue
                t_tot = t-t0
                uptime[d_true][idx].append( 100*(t_tot-timeout_acc)/t_tot )


    print('Range | ID |   0   45   90   135  180 |')
    print('------|----|--------------------------|')
    print(' 2m   | 0  | {:3.0f}% {:3.0f}% {:3.0f}% {:3.0f}% {:3.0f}% |'.format(*uptime['2m'][0]))
    print('      | 1  | {:3.0f}% {:3.0f}% {:3.0f}% {:3.0f}% {:3.0f}% |'.format(*uptime['2m'][1]))
    print('------|----|--------------------------|')
    print(' 7m   | 0  | {:3.0f}% {:3.0f}% {:3.0f}% {:3.0f}% {:3.0f}% |'.format(*uptime['7m'][0]))
    print('      | 1  | {:3.0f}% {:3.0f}% {:3.0f}% {:3.0f}% {:3.0f}% |'.format(*uptime['7m'][1]))
    print('---------------------------------------\n')


def stats_calculate(data):
    stats = dict()
    for mnt in MOUNTING:
        stats[mnt] = dict()
        for tx_mode in TX_MODES:
            stats[mnt][tx_mode] = dict()
            for d_true in RANGES:
                stats[mnt][tx_mode][d_true] = dict()
                for angle in TAG_ANGLES:
                    stats[mnt][tx_mode][d_true][angle] = [dict() for idx in range(N_ANCHORS)]
                    for idx in range(N_ANCHORS):
                        data_d = data[mnt][tx_mode][d_true]['dist'][angle][idx]
                        data_e = data[mnt][tx_mode][d_true]['dist_error'][angle][idx]
                        data_t = data[mnt][tx_mode][d_true]['time'][angle][idx]
                        
                        N = len(data_d)
                        d_mean = lps_stats.mean(data_d)
                        d_std = lps_stats.stdev(data_d)
                        d_out = lps_stats.outlier_count(data_d, [d_mean-OUTLIER_THRESHOLD, d_mean+OUTLIER_THRESHOLD])
                        e_mean = lps_stats.mean(data_e)
                        t_up = lps_stats.uptime(data_t, TIMEOUT_THRESHOLD)


                        stats[mnt][tx_mode][d_true][angle][idx] = {
                            'N': N,
                            'e_mean': e_mean,
                            'd_mean': d_mean,
                            'd_std': d_std,
                            'd_outlier_cnt': d_out,
                            'd_outlier_perc': d_out/N,
                            'uptime_perc': t_up,
                            }
    return stats

# get list of statistical value of several logs
def get_stat(stat_dict, stat_name, *, mounting=MOUNTING, tx_mode=TX_MODES, d_true=RANGES, angle=TAG_ANGLES, idx=range(N_ANCHORS)):
    return_array = []
    
    for mnt in mounting:
        for txm in tx_mode:
            for d in d_true:
                for ang in angle:
                    for i in idx:
                        return_array.append(stat_dict[mnt][txm][d][ang][i][stat_name])
    return return_array

def stats_print_comparison(stats):
    print('********************************')
    print('*  Ranging Stats               *')
    print('********************************')

    # Mounting
    variables = {'MOUNTING': MOUNTING,
                'TX_MODE': TX_MODES,
                'RANGE': RANGES,
                'ANGLES': TAG_ANGLES,
    }
    
    for title, var_list in variables.items():
        print('\n>> {}'.format(title))
        print('      Uptime, Outlier, Mean Std, Precision')

        for val in var_list:
            if title == 'MOUNTING':
                kwargs = {'mounting': [val]}
            elif title == 'TX_MODE':
                kwargs = {'tx_mode': [val]}
            elif title == 'RANGE':
                kwargs = {'d_true': [val]}
            elif title == 'ANGLES':
                kwargs = {'angle': [val]}
            else:
                continue
            

            uptime = np.mean(get_stat(stats, 'uptime_perc', **kwargs))
            out_perc = np.mean(get_stat(stats, 'd_outlier_perc', **kwargs))
            d_std = np.mean(get_stat(stats, 'd_std', **kwargs))
            e_var = np.std(get_stat(stats, 'e_mean', **kwargs))

            print('{:3s}:  {:5.2f}%,  {:5.2f}%,   {:4.3f}m,    {:4.3f}m'.format(val, uptime, out_perc, d_std, e_var))


if __name__ == '__main__':
    data = data_bin_init()
    load_data(data_path, data)
    stats_uptime_print(data)
    
    statistics = stats_calculate(data)
    #stat_ranging_boxplots(data)
    stats_print_comparison(statistics)



