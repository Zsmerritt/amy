#!/usr/bin/env python3
"""Reconstruct the GM program/drum-note mapping for Leeman's 1093-preset
8-font AMY bank from pcm_tiny.h alone.

Structure (verified by inspection): six composite font blocks, each =
melodic zones in GM program order (guitar/bass families sourced from
specialist fonts, hence the tag runs) followed by that font's drum kit in
GM drum-note order. Zones for one program are consecutive (multisample
zones differ only by a trailing note/number suffix).

Melodic: group consecutive same-tag zones by base name, then align the
group sequence against GM programs 0..127 with a monotone DP whose score
is a keyword match between the sample name and the program identity.
Drums: align each drum run against GM notes 27..87 the same way.
"""
import json
import re
import sys

S = sys.argv[1]

ENT = re.findall(r'\{(\d+), (\d+), (\d+), (\d+), (\d+)\},\s*/\* \[(\d+)\] (.*?) \*/',
                 open(S + '/big_pcm_tiny.h', encoding='utf-8').read())

def nametag(c):
    m = re.match(r'(.*) \((\w+)\)$', c)
    return (m.group(1).strip(), m.group(2)) if m else (c.strip(), '?')

zones = []
for off, length, ls, le, root, idx, comment in ENT:
    name, tag = nametag(comment)
    zones.append({'i': int(idx), 'root': int(root), 'name': name, 'tag': tag,
                  'off': int(off), 'len': int(length),
                  'loop': int(le) > int(ls) and int(le) < int(length)})

# ---- split into blocks: a block ends where a drum run ends ----
blocks = []
cur = []
for j, z in enumerate(zones):
    cur.append(z)
    if z['tag'] == 'drum' and (j + 1 == len(zones) or zones[j+1]['tag'] != 'drum'):
        blocks.append(cur)
        cur = []
if cur:
    blocks.append(cur)

# ---- base-name grouping ----
NOTE_SUFFIX = re.compile(
    r'([ _\-]?[A-G][#b]?[ _\-]?\-?\d{1,2}[_ ]*$)|([ _\-]?#?\d{1,3}[_ ]*$)|([ _\-]+[RL]$)')
def base_name(n):
    prev = None
    n = n.strip()
    while prev != n:
        prev = n
        n = NOTE_SUFFIX.sub('', n).strip()
    return n.lower() or prev.lower()

def alpha_key(n):
    return re.sub(r'[^a-z]', '', n.lower())

def alpha_key_early(n):
    """Alpha key of the name up to (and excluding) any file-ish suffix, so
    'HARPS134.134.L08' and 'HARPS134.134S.L08' collide."""
    return re.sub(r'[^a-z]', '', n.lower().split('.')[0])

def lcs_len(a, b):
    """Longest common substring length."""
    if not a or not b:
        return 0
    prev = [0] * (len(b) + 1)
    best = 0
    for i in range(1, len(a) + 1):
        cur = [0] * (len(b) + 1)
        for j in range(1, len(b) + 1):
            if a[i-1] == b[j-1]:
                cur[j] = prev[j-1] + 1
                if cur[j] > best:
                    best = cur[j]
        prev = cur
    return best

def similar(a, b):
    ka, kb = alpha_key(a), alpha_key(b)
    if ka == kb:
        return True
    m = lcs_len(ka, kb)
    return m >= 4 and m >= 0.6 * min(len(ka), len(kb))

def group_melodic(block):
    groups = []
    for z in block:
        if z['tag'] == 'drum':
            continue
        b = base_name(z['name'])
        if groups and groups[-1]['tag'] == z['tag'] and \
                (groups[-1]['base'] == b or similar(groups[-1]['base'], b)):
            groups[-1]['zones'].append(z)
        else:
            groups.append({'base': b, 'tag': z['tag'], 'zones': [z]})
    return groups

# ---- GM program keyword table -----------------------------------------------
# score(name, program): higher = better. Keywords chosen for THESE fonts'
# naming styles (E-mu 8.3 names, Power-GM style, Vintage-Dreams style).
KW = {
0: ['grand', 'piano', 'ap_', 'acpiano', 'kpiano', 'piano1'],
1: ['brite', 'bright', 'kpiano', 'piano2'],
2: ['elec grand', 'elecgrand', 'e.grand', 'egrand', 'cp7', 'cp8'],
3: ['honky', 'tack'],
4: ['rhodes', 'ep_rho', 'epa', 'tine', 'wurli', 'e.piano1', 'ep1', 'ep a', 'wurli', 'rhod'],
5: ['dx', 'fm ep', 'ep_dx', 'e.piano2', 'ep2', 'rhod? no'],
6: ['harpsi', 'harps', 'hcord', 'cembalo'],
7: ['clav', 'clv'],
8: ['celes', 'celest'],
9: ['glock', 'glok'],
10: ['music box', 'musicbox', 'musbox', 'orgel'],
11: ['vibe', 'vibra'],
12: ['marim'],
13: ['xylo'],
14: ['tubular', 'tbell', 'tub', 'chime', 'bell'],
15: ['dulcimer', 'dlcmr', 'dul', 'santur'],
16: ['drawbar', 'tonewheel', 'organ 1', 'org 1', 'b3', 'jazz org', 'draw', 'ky_ojz'],
17: ['perc org', 'percussive', 'organ 2', 'org 2', 'full slow', 'perc'],
18: ['rock org', 'rock_or', 'full fast'],
19: ['pipe', 'church'],
20: ['reed org', 'read_org', 'reed', 'read_org'],
21: ['accord', 'pump', 'accor', 'ky_acc'],
22: ['harmon', 'harmonica'],   # careful: 'harmon mute' is program 59
23: ['tango', 'bandneon', 'bandoneon', '+accordion', 'tang'],
24: ['nylon', 'n guitar', 'n gtr', 'classic gtr', 'gt_enl'],
25: ['steel', 'stlgtr', 'ac gtr', 'acgtr', 'folk', 'mart', 'gt_est'],
26: ['jazz g', 'gibso', 'jazzgtr', 'gt_ejz', 'jazz_g'],
27: ['clean', 'strat', 'electric guitar', 'eg_st', 'strat'],
28: ['mute', 'gtr mute', 'muted gtr', 'mutegtr', 'gt_emt'],
29: ['overdrive', 'ovrdrive', 'odgtr', 'srv', 'gt_eod', 'eg_od'],
30: ['dist', 'distgtr', 'gt_eds'],
31: ['harmonics', 'harmnics', 'gt_ehr'],
32: ['acoustic bass', 'ba_upr', 'upright', 'acbass', 'a.bass'],
33: ['finger', 'fingbas', 'fbass', 'ba_elb_fi'],
34: ['pick', 'pickbass', 'pbass', 'ba_elb_pu', 'eb_pk'],
35: ['fretless', 'frtless', 'eb_fr'],
36: ['slap bass 1', 'slap1', 'slapbas1', 'slap', 'eb_sl'],
37: ['slap bass 2', 'slap2', 'slapbas2', 'syn_b1'],
38: ['syn bass 1', 'synbass1', 'synbas1', 'synth bass 1', 'sy_bas', 'tb 303', 'tb303'],
39: ['syn bass 2', 'synbass2', 'synbas2', 'synth bass 2', 'sawbass'],
40: ['violin'],
41: ['viola', 'st_vas', 'arcoviolin'],
42: ['cello', 'arcocello', 'cello'],
43: ['contra', 'ctrabass', 'double bass', 'bowba', 'bowba', 'contravio'],
44: ['tremolo', 'trm_str', 'trem'],
45: ['pizz'],
46: ['harp', 'fi_hrp', 'troubh', 'harp_'],
47: ['timp', 'tp_tmp'],
48: ['strings', 'st_', 'vlns', 'marcato', 'str ', 'st_ste', 'vlns', 'g1 vlns'],
49: ['slow str', 'slowstr', 'legato'],
50: ['syn str 1', 'synstr1', 'syn strings', 'synstr', 'syn_p1', 'synstr'],
51: ['syn str 2', 'synstr2'],
52: ['choir', 'aah', 'ahh', 'chr'],
53: ['ooh', 'voice ooh', 'doo'],
54: ['syn vox', 'synvox', 'vox', 'sy_gs2'],
55: ['orch hit', 'orchhit', 'hit', 'orch_'],
56: ['trumpet', 'tp_', 'tpt', 'tp_2c', 'tp_'],
57: ['trombone', 'tbone', 'trb'],
58: ['tuba'],
59: ['harmon mute', 'muted tp', 'mute tp', 'mute trumpet', 'harmon_', 'harmon m'],
60: ['french', 'frhorn', 'fr horn', 'horns', 'horn_'],
61: ['brass', 'br_tp'],
62: ['syn brass 1', 'synbrass1', 'synbrs', 'syn_bra'],
63: ['syn brass 2', 'synbrass2'],
64: ['soprano', 'ww_sxs', 'sop_sax'],
65: ['alto', 'ww_sxa'],
66: ['tenor', 'ww_sxt', 'tenor'],
67: ['bari', 'saxba', 'ww_sxb'],
68: ['oboe'],
69: ['english', 'eng horn', 'ww_ehr', 'engli'],
70: ['bassoon', 'basoon', 'fag_', 'fag'],
71: ['clarinet', 'clar', 'ww_cla', 'clarine'],
72: ['piccolo', 'picc'],
73: ['flute', 'ww_flu'],
74: ['recorder', 'rcrder', 'ww_res'],
75: ['pan', 'panflute', 'panpipe', 'fi_pan'],
76: ['bottle', 'blow'],
77: ['shaku', 'shakuhac'],
78: ['whistle'],
79: ['ocarina', 'ocrina'],
80: ['square', 'sq', 'sqr'],
81: ['saw lead', 'obxsaw', 'saw', 'mini saw'],
82: ['calliope', 'cliope'],
83: ['chiff'],
84: ['charang', 'chrang'],
85: ['solo vox', 'voice lead'],
86: ['fifth', '5th'],
87: ['bass lead', 'bass&lead', 'bass & lead'],
88: ['fantasia', 'new age', 'newage'],
89: ['warm pad', 'warm', 'sy_p50'],
90: ['polysynth', 'poly', 'sy_pad'],
91: ['space voice', 'choir pad', 'halo? no'],
92: ['bowed', 'glass'],
93: ['metal pad', 'metallic'],
94: ['halo'],
95: ['sweep'],
96: ['ice rain', 'rain'],
97: ['soundtrack', 'strack'],
98: ['crystal', 'crystl', 'obx fm bell'],
99: ['atmosphere', 'atmos'],
100: ['brightness', 'brite pad'],
101: ['goblin'],
102: ['echo', 'echo drops', 'drops'],
103: ['sci-fi', 'star theme', 'startheme', 'scifi'],
104: ['sitar', 'fi_sit'],
105: ['banjo', 'bnjo', 'fi_bnj'],
106: ['shamisen', 'shamsen', 'shamisen'],
107: ['koto'],
108: ['kalimba', 'klimba'],
109: ['bagpipe', 'bag', 'fi_bag', 'bagpipe'],
110: ['fiddle'],
111: ['shanai', 'shenai', 'shnai', 'shanai'],
112: ['tinkle', 'tinker'],
113: ['agogo'],
114: ['steel drum', 'steeldrm', 'stldrums', 'tp_sdr', 'steeldr'],
115: ['woodblock', 'wood block', 'woodblk', 'woodbloc'],
116: ['taiko', 'taiko'],
117: ['melodic tom', 'melo tom', 'mtom', 'tom_jr', 'melotom'],
118: ['syn drum', 'syndrum', 'synth drum', 'tom_t'],
119: ['reverse', 'revcym', 'rev cym', 'dr_ccr', 'revcymb'],
120: ['fret noise', 'fretnoise', 'fret'],
121: ['breath', 'brth'],
122: ['seashore', 'sea', 'shore', 'wave'],
123: ['bird', 'tweet'],
124: ['telephone', 'phone', 'telphone'],
125: ['helicopter', 'heli', 'copter'],
126: ['applause', 'applse', 'clap? no'],
127: ['gunshot', 'gun'],
}
# Family ranges implied by tags: guitar-font zones are programs 24..31,
# bass-font zones 32..39 (plus low string/brass doubles seen in the dumps).
TAG_RANGE = {'guitar': (24, 31), 'bass': (32, 63)}

# ---- anchors ----------------------------------------------------------------
# (1) Leading 3-digit numbers in sample names are 1-based GM program numbers
#     (e.g. "081square.2_f#.l" -> program 80, "024tangaoord" -> 23).
def digit_anchor(base):
    m = re.match(r'0?(\d{3})', base)
    if m:
        p = int(m.group(1)) - 1
        if 0 <= p <= 127:
            return p
    return None

# (2) The 160-preset bank (gm2350) was merged from the same source fonts and
#     its program mapping is known: same sample name => same program.
GM_PROGRAM_PRESET = [
      0,  0,  1,  0,  2,  3,  4,  5,  6,  7,  6,  8,  9, 10, 11, 12,
     13, 14, 15, 16, 17, 18, 19, 17, 20, 21, 22, 23, 24, 25, 26, 27,
     28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43,
     44, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 57,
     58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73,
     74, 57, 75, 75, 26, 76, 77, 78,  3, 44, 45, 76, 45, 46, 45, 45,
     77, 79, 80, 20, 45, 81, 49, 23, 82, 83, 84, 85, 86, 87, 36, 88,
     89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99,100,101,102,103,104,
]
_preset_to_prog = {}
for _p in range(127, -1, -1):
    _preset_to_prog[GM_PROGRAM_PRESET[_p]] = _p
SMALL = re.findall(r'\{(\d+), (\d+), (\d+), (\d+), (\d+)\},\s*/\* \[(\d+)\] (.*?) \*/',
                   open(S + '/gm2350_pcm_tiny.h', encoding='utf-8').read())
ANCHOR = {}
for _e in SMALL:
    _n, _t = nametag(_e[6])
    _idx = int(_e[5])
    if _idx in _preset_to_prog:
        ANCHOR[alpha_key_early(_n)] = _preset_to_prog[_idx]

def kw_score(base, prog):
    da = digit_anchor(base)
    if da is not None:
        return 1000 if da == prog else -200
    ap = ANCHOR.get(alpha_key_early(base))
    if ap is not None:
        return 800 if ap == prog else -100
    s = 0
    for k in KW[prog]:
        if ' no' in k:
            continue
        if k in base:
            s = max(s, 10 * min(len(k), 12))
    return s

def align(groups, lo=0, hi=127):
    """Monotone DP over (group index, program index): each group is either
    assigned a program (strictly increasing across groups) or dropped with a
    penalty; programs may be skipped freely. Tag family ranges give a bonus,
    not a hard constraint (the baker mis-tags a few, e.g. TB-303 as guitar)."""
    n = len(groups)
    progs = list(range(lo, hi + 1))
    m = len(progs)
    NEG = float('-inf')
    DROP = -50
    dp = [[NEG] * (m + 1) for _ in range(n + 1)]
    back = [[None] * (m + 1) for _ in range(n + 1)]
    for j in range(m + 1):
        dp[0][j] = 0.0
    for i in range(1, n + 1):
        g = groups[i-1]
        glo, ghi = TAG_RANGE.get(g['tag'], (0, 127))
        for j in range(m + 1):
            # drop this group entirely
            if dp[i-1][j] > NEG and dp[i-1][j] + DROP > dp[i][j]:
                dp[i][j] = dp[i-1][j] + DROP
                back[i][j] = ('drop',)
            if j == 0:
                continue
            # leave program progs[j-1] unused
            if dp[i][j-1] > dp[i][j]:
                dp[i][j] = dp[i][j-1]
                back[i][j] = ('skip',)
            # assign group i-1 -> program progs[j-1]
            p = progs[j-1]
            sc = kw_score(g['base'], p)
            bonus = 15 if glo <= p <= ghi and g['tag'] in TAG_RANGE else 0
            if dp[i-1][j-1] > NEG:
                val = dp[i-1][j-1] + sc + bonus - (3 if sc == 0 else 0)
                if val > dp[i][j]:
                    dp[i][j] = val
                    back[i][j] = ('take', sc)
            # share program progs[j-1] with the previous group (extra zones of
            # the same instrument that the merger didn't catch)
            if dp[i-1][j] > NEG:
                val = dp[i-1][j] + sc + bonus - 8
                if val > dp[i][j]:
                    dp[i][j] = val
                    back[i][j] = ('share', sc)
    # walk back
    out = [None] * n
    i, j = n, m
    while i > 0:
        b = back[i][j]
        if b is None:
            if j > 0:
                j -= 1
                continue
            i -= 1     # unreachable state; bail upward
            continue
        if b[0] == 'skip':
            j -= 1
        elif b[0] == 'drop':
            out[i-1] = None
            i -= 1
        elif b[0] == 'share':
            out[i-1] = (progs[j-1], b[1])
            i -= 1
        else:
            out[i-1] = (progs[j-1], b[1])
            i -= 1
            j -= 1
    return out

# ---- drums -------------------------------------------------------------------
DRUM_KW = {
27:['high q','hi q','hi-q','filter snap','filtersnap','q_s'],28:['slap'],
29:['scratch push','push','vinyl','scratch1','sc_push'],30:['scratch pull','pull','scratch2'],
31:['stick'],32:['square click','sq click','click'],33:['metronome','met click'],
34:['met bell','metronome bell'],35:['kick 2','bd 2','acou bd','kick2','bassdrum','bd1','bd'],
36:['kick 1','bd1','kick','bd','909bd','808bd'],37:['side stick','sidestick','rim','rimshot'],
38:['snare 1','snare','sd','acou sd'],39:['clap','handclap','hand clap'],
40:['snare 2','elec sd','esnare','sd2'],41:['low floor','floor tom','lf tom','floortom'],
42:['closed h','ch ','chh','closed hat','hh closed','clhat','close'],
43:['high floor','hf tom','floortombrite','floor'],44:['pedal h','phh','pedal'],
45:['low tom','ltom'],46:['open h','ohh','open hat','ophat'],47:['low-mid','lm tom','mid tom'],
48:['hi-mid','hi mid','hm tom','hitom'],49:['crash 1','crash','cr1'],50:['high tom','htom','hi tom'],
51:['ride 1','ride','ping'],52:['china','chinacrash'],53:['ride bell','bell'],
54:['tambourine','tamb'],55:['splash'],56:['cowbell','cow'],57:['crash 2','crash cymbal','cr2','med crash'],
58:['vibraslap','vibra loop','vslap'],59:['ride 2','ride cym'],60:['hi bongo','h bongo','bongo'],
61:['low bongo','l bongo','m bongo'],62:['mute hi conga','mute conga','quinto closed','quintoclosed','mtconga'],
63:['open hi conga','hi conga','quinto tone','conga'],64:['low conga','tumba'],
65:['high timbale','timbale strike','timbale hi'],66:['low timbale','timbale rim','timbale low','timbale'],
67:['high agogo','agogo bell','agogo'],68:['low agogo'],69:['cabasa'],70:['maracas'],
71:['short whistle','samba whistle','whistle'],72:['long whistle'],73:['short guiro','guiro down','guiro'],
74:['long guiro','guiro up'],75:['claves','clave'],76:['hi wood','wood block','woodblock'],
77:['low wood'],78:['mute cuica','quica hi','cuica','quica'],79:['open cuica','quica down'],
80:['mute triangle','mute tri'],81:['open triangle','triangle'],82:['shaker','cabasa2'],
83:['jingle','sleigh'],84:['bell tree','belltree','tree'],85:['castanet'],
86:['mute surdo','surdo mute','taiko? no','pc_sur'],87:['open surdo','surdo','taiko'],
}
def drum_kw_score(name, note):
    n = name.lower()
    s = 0
    for k in DRUM_KW[note]:
        if ' no' in k:
            continue
        if k in n:
            s = max(s, 10 * min(len(k), 12))
    return s

def align_drums(drumzones):
    notes = list(range(27, 88))
    n, m = len(drumzones), len(notes)
    NEG = -10**9
    dp = [[NEG]*(m+1) for _ in range(n+1)]
    back = [[None]*(m+1) for _ in range(n+1)]
    for j in range(m+1):
        dp[0][j] = 0
    for i in range(1, n+1):
        z = drumzones[i-1]
        for j in range(m+1):
            if dp[i-1][j] > NEG and dp[i-1][j] - 50 > dp[i][j]:
                dp[i][j] = dp[i-1][j] - 50
                back[i][j] = ('drop',)
            if j == 0:
                continue
            if dp[i][j-1] > dp[i][j]:
                dp[i][j] = dp[i][j-1]
                back[i][j] = ('skip',)
            if dp[i-1][j-1] > NEG:
                sc = drum_kw_score(z['name'], notes[j-1])
                val = dp[i-1][j-1] + sc - (3 if sc == 0 else 0)
                if val > dp[i][j]:
                    dp[i][j] = val
                    back[i][j] = ('take', sc)
    out = [None]*n
    i, j = n, m
    while i > 0:
        b = back[i][j]
        if b is None:
            if j > 0:
                j -= 1
                continue
            i -= 1
            continue
        if b[0] == 'skip':
            j -= 1
        elif b[0] == 'drop':
            out[i-1] = None
            i -= 1
        else:
            out[i-1] = (notes[j-1], b[1])
            i -= 1
            j -= 1
    return out

# ---- run ---------------------------------------------------------------------
result = []
for bi, block in enumerate(blocks):
    groups = group_melodic(block)
    asg = align(groups)
    drums = [z for z in block if z['tag'] == 'drum']
    dasg = align_drums(drums)
    result.append({'block': bi, 'groups': groups, 'assign': asg,
                   'drums': drums, 'drum_assign': dasg})
    print('=== BLOCK %d: %d melodic groups, %d drum zones' %
          (bi, len(groups), len(drums)))
    for g, a in zip(groups, asg):
        zs = g['zones']
        flag = '' if (a and a[1] > 0) else '  ???'
        print('  prog %s  x%d roots %-16s %-8s %s%s' %
              (('%3d' % a[0]) if a else ' - ', len(zs),
               ','.join(str(z['root']) for z in zs)[:16], g['tag'], g['base'][:34], flag))
    print('  -- drums --')
    for z, a in zip(drums, dasg):
        flag = '' if (a and a[1] > 0) else '  ???'
        print('  note %s root %3d  %s%s' %
              (('%3d' % a[0]) if a else ' - ', z['root'], z['name'][:36], flag))

json.dump([{'block': r['block'],
            'melodic': [{'base': g['base'], 'tag': g['tag'],
                         'zones': [z['i'] for z in g['zones']],
                         'roots': [z['root'] for z in g['zones']],
                         'prog': (a[0] if a else None), 'score': (a[1] if a else 0)}
                        for g, a in zip(r['groups'], r['assign'])],
            'drums': [{'i': z['i'], 'root': z['root'], 'name': z['name'],
                       'note': (a[0] if a else None), 'score': (a[1] if a else 0)}
                      for z, a in zip(r['drums'], r['drum_assign'])]}
           for r in result],
          open(S + '/gm_big_map_proposal.json', 'w'), indent=1)
print('WROTE', S + '/gm_big_map_proposal.json')
