#include "../Include/CMrcFileInc.h"
#include <Util/Util_SwapByte.h>
#include <memory.h>

using namespace Mrc;

CMainHeader::CMainHeader(void)
{
	memset(this, 0, sizeof(CMainHeader));
}

CMainHeader::~CMainHeader(void)
{
}

void CMainHeader::SwapByte(void)
{
	nx = Util_SwapByte::DoIt(nx);			
	ny = Util_SwapByte::DoIt(ny);			
	nz = Util_SwapByte::DoIt(nz);			
	mode = Util_SwapByte::DoIt(mode);		
	nxstart = Util_SwapByte::DoIt(nxstart);	
	nystart = Util_SwapByte::DoIt(nystart);
	nzstart = Util_SwapByte::DoIt(nzstart);	
	mx = Util_SwapByte::DoIt(mx);		
	my = Util_SwapByte::DoIt(my);		
	mz = Util_SwapByte::DoIt(mz);	
	xlen = Util_SwapByte::DoIt(xlen);	
	ylen = Util_SwapByte::DoIt(ylen);	
	zlen = Util_SwapByte::DoIt(zlen);	
	alpha = Util_SwapByte::DoIt(alpha);	
	beta = Util_SwapByte::DoIt(beta);	
	gamma = Util_SwapByte::DoIt(gamma);	
	mapc = Util_SwapByte::DoIt(mapc);
	mapr = Util_SwapByte::DoIt(mapr);	
	maps = Util_SwapByte::DoIt(maps);		
	amin = Util_SwapByte::DoIt(amin);		
	amax = Util_SwapByte::DoIt(amax);			
	amean = Util_SwapByte::DoIt(amean);		
	ispg = Util_SwapByte::DoIt(ispg);			
	nsymbt = Util_SwapByte::DoIt(nsymbt);		
	dvid = Util_SwapByte::DoIt(dvid);			
	nblank = Util_SwapByte::DoIt(nblank);			
	itst = Util_SwapByte::DoIt(itst);		
	numintegers = Util_SwapByte::DoIt(numintegers);
	numfloats = Util_SwapByte::DoIt(numfloats);	
	sub = Util_SwapByte::DoIt(sub);		
	zfac = Util_SwapByte::DoIt(zfac);		
	min2 = Util_SwapByte::DoIt(min2);		
	max2 = Util_SwapByte::DoIt(max2);		
	min3 = Util_SwapByte::DoIt(min3);		
	max3 = Util_SwapByte::DoIt(max3);		
	min4 = Util_SwapByte::DoIt(min4);		
	max4 = Util_SwapByte::DoIt(max4);		
	type = Util_SwapByte::DoIt(type);		
	lensnum = Util_SwapByte::DoIt(lensnum);	
	nd1 = Util_SwapByte::DoIt(nd1);		
	nd2 = Util_SwapByte::DoIt(nd2);		
	vd1 = Util_SwapByte::DoIt(vd1);		
	vd2 = Util_SwapByte::DoIt(vd2);		
	min5 = Util_SwapByte::DoIt(min5);		
	max5 = Util_SwapByte::DoIt(max5);		
	numtimes = Util_SwapByte::DoIt(numtimes);	
	imgseq = Util_SwapByte::DoIt(imgseq);		
	xtilt = Util_SwapByte::DoIt(xtilt);		
	ytilt = Util_SwapByte::DoIt(ytilt);		
	ztilt = Util_SwapByte::DoIt(ztilt);		
	numwaves = Util_SwapByte::DoIt(numwaves);	
	wave1 = Util_SwapByte::DoIt(wave1);		
	wave2 = Util_SwapByte::DoIt(wave2);	
	wave3 = Util_SwapByte::DoIt(wave3);		
	wave4 = Util_SwapByte::DoIt(wave4);		
	wave5 = Util_SwapByte::DoIt(wave5);	
	z0 = Util_SwapByte::DoIt(z0);			
	x0 = Util_SwapByte::DoIt(x0);		
	y0 = Util_SwapByte::DoIt(y0);		
	nlabl = Util_SwapByte::DoIt(nlabl);		
}
