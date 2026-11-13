package com.lineforge.lfr;
import java.util.*;
public class ProtocolSelfTest {
  static class Collector implements Protocol.Listener {
    List<Integer> types=new ArrayList<>(), replies=new ArrayList<>();
    byte[] status, cal;
    int ignored=0, crcErrors=0;
    @Override public void onPacket(int t,byte[] p){types.add(t);if(t==Protocol.STATUS)status=p;if(t==Protocol.CALIBRATION)cal=p;}
    @Override public void onLegacyStatus(byte[] s){status=s;types.add(0xD0);}
    @Override public void onReply(int b){replies.add(b);}
    @Override public void onDiscarded(int n,boolean bad){if(bad)crcErrors++;else ignored+=n;}
  }
  static void yes(boolean cond,String msg){if(!cond)throw new RuntimeException(msg);}
  public static void main(String[] args) {
    Random rnd=new Random(42);
    Collector c=new Collector(); Protocol.Parser p=new Protocol.Parser(c);
    byte[] stat={40,0,0,0,1,0,1,0};
    byte[] valid=Protocol.frame(Protocol.STATUS,7,stat);
    byte[] noise={(byte)0xFB,(byte)0xEA,(byte)0x00,(byte)0xD4,(byte)0xCC};
    p.feed(noise,noise.length);
    for(int i=0;i<valid.length;i++)p.feed(new byte[]{valid[i]},1);
    yes(c.types.size()==1 && c.types.get(0)==Protocol.STATUS,"framed status not recovered");
    yes(Arrays.equals(c.status,stat),"PID incorrect");
    // corrupted status, followed by good telemetry despite random read boundaries.
    byte[] wrong=valid.clone();wrong[wrong.length-1]^=3;
    byte[] levels=new byte[13];for(int i=0;i<8;i++)levels[i]=(byte)(i*30);levels[8]=(byte)0xAC;levels[9]=0x0D;levels[10]=0x07;
    byte[] sensor=Protocol.frame(Protocol.SENSORS,8,levels);
    byte[] test=new byte[wrong.length+sensor.length];System.arraycopy(wrong,0,test,0,wrong.length);System.arraycopy(sensor,0,test,wrong.length,sensor.length);
    int at=0;while(at<test.length){int n=Math.min(test.length-at,1+rnd.nextInt(5));p.feed(Arrays.copyOfRange(test,at,at+n),n);at+=n;}
    yes(c.crcErrors>0,"corruption not counted");yes(c.types.contains(Protocol.SENSORS),"failed to recover after bad frame");
    byte[] cal=new byte[32];for(int i=0;i<32;i++)cal[i]=(byte)(i+20);
    byte[] cf=Protocol.frame(Protocol.CALIBRATION,9,cal);
    p.feed(cf,cf.length);yes(Arrays.equals(c.cal,cal),"bad calibration decode");
    // original 9 byte status with XOR still accepted for rollback firmware.
    byte[] legacy=new byte[9];legacy[0]=(byte)0xD0;System.arraycopy(stat,0,legacy,1,7);
    int crc=0;for(int i=1;i<8;i++)crc^=(legacy[i]&255);legacy[8]=(byte)crc;
    p.feed(legacy,legacy.length);yes(c.types.contains(0xD0),"legacy status rejected");
    p.feed(new byte[]{0x06,0x15,(byte)0xC1},3);yes(c.replies.size()==3,"one-byte replies invalid");
    yes(Protocol.frame(Protocol.SENSORS,1,new byte[13]).length==19,"telemetry frame size");
    System.out.println("PASS: garbage recovery, CRC failure, fragmented stream, 19B telemetry, 32B calibration, legacy status, ACKs");
  }
}
