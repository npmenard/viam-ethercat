import struct, sys, collections

CMD = {0:'NOP',1:'APRD',2:'APWR',3:'APRW',4:'FPRD',5:'FPWR',6:'FPRW',7:'BRD',
       8:'BWR',9:'BRW',10:'LRD',11:'LWR',12:'LRW',13:'ARMW',14:'FRMW'}

def parse(path):
    d = open(path,'rb').read()
    magic = struct.unpack('<I', d[:4])[0]
    nsec = (magic == 0xa1b23c4d)
    off = 24
    frames = []  # (t, [(cmd, adp, ado, dlen, wkc)])
    while off + 16 <= len(d):
        ts_sec, ts_frac, incl, orig = struct.unpack('<IIII', d[off:off+16]); off += 16
        pkt = d[off:off+incl]; off += incl
        t = ts_sec + ts_frac/(1e9 if nsec else 1e6)
        if len(pkt) < 16 or pkt[12:14] != b'\x88\xa4': continue
        ec = struct.unpack('<H', pkt[14:16])[0]; eclen = ec & 0x7ff
        p = 16; dgs = []
        while p + 10 <= len(pkt):
            cmd, idx = pkt[p], pkt[p+1]
            adp, ado = struct.unpack('<HH', pkt[p+2:p+6])  # 4-byte address @ p+2 (was p+4: shifted past LEN)
            lf = struct.unpack('<H', pkt[p+6:p+8])[0]; dlen = lf & 0x7ff; more = (lf>>15)&1  # LEN @ p+6 (was p+8 = IRQ -> dlen always 0)
            p += 10
            wkc = struct.unpack('<H', pkt[p+dlen:p+dlen+2])[0] if p+dlen+2<=len(pkt) else -1
            dgs.append((cmd, adp, ado, dlen, wkc))
            p += dlen + 2
            if not more: break
        frames.append((t, dgs))
    return frames

f = parse(sys.argv[1])
print(f"=== {sys.argv[1]}: {len(f)} frames ===")
if not f: sys.exit()
dur = f[-1][0]-f[0][0]
print(f"duration {dur:.3f}s, avg {len(f)/dur:.0f} frames/s")
# datagram cmd histogram
hist = collections.Counter()
for _,dgs in f:
    for cmd,adp,ado,dlen,wkc in dgs: hist[CMD.get(cmd,hex(cmd))]+=1
print("datagram types:", dict(hist))
# LRW (process data) inter-frame gaps
lrw_t = [t for t,dgs in f if any(cmd==12 for cmd,_,_,_,_ in dgs)]
print(f"LRW frames: {len(lrw_t)}")
if len(lrw_t)>2:
    gaps = [(lrw_t[i+1]-lrw_t[i])*1000 for i in range(len(lrw_t)-1)]
    gaps_sorted = sorted(gaps, reverse=True)
    print(f"LRW gap ms: max={gaps_sorted[0]:.2f} 2nd={gaps_sorted[1]:.2f} 3rd={gaps_sorted[2]:.2f} | median={sorted(gaps)[len(gaps)//2]:.3f}")
    # locate the largest gap and what brackets it
    imax = gaps.index(gaps_sorted[0])
    gt = lrw_t[imax]-f[0][0]
    print(f"  largest LRW gap {gaps_sorted[0]:.2f}ms at t+{gt:.3f}s")
    # count LRW gaps > 5ms (potential watchdog territory)
    big = [(g, (lrw_t[i]-f[0][0])) for i,g in enumerate(gaps) if g>5]
    print(f"  LRW gaps >5ms: {len(big)}  ", [(f'{g:.1f}ms@{tt:.2f}s') for g,tt in big[:12]])
# DC datagrams: ARMW/FRMW (reference clock distribution) on 0x0910
dc = [t for t,dgs in f if any(cmd in (13,14) for cmd,_,_,_,_ in dgs)]
print(f"DC (ARMW/FRMW) frames: {len(dc)}  (per-cycle reference-clock distribution if ~= LRW count)")

# timeline: frame-type counts per 500ms bin
print("\n--- timeline (500ms bins): LRW / BRD / FPRD / FPWR ---")
t0 = f[0][0]; bins = collections.defaultdict(lambda: collections.Counter())
for t,dgs in f:
    b = int((t-t0)/0.5)
    for cmd,_,_,_,_ in dgs: bins[b][CMD.get(cmd,hex(cmd))]+=1
for b in sorted(bins):
    c=bins[b]; print(f"  t+{b*0.5:4.1f}s: LRW={c['LRW']:5d} BRD={c['BRD']:5d} FPRD={c['FPRD']:3d} FPWR={c['FPWR']:3d}")
