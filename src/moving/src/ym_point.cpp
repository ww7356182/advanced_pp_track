#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include<canlib.h>
#include<stdio.h>
#include<Eigen/Core>
#include<Eigen/Geometry>
//#include "LinearMath/btMatrix3x3.h"

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_path= n_.advertise<nav_msgs::Path>("/path_ym", 1,true);
    marker_pub=n_.advertise<visualization_msgs::MarkerArray>("target_point", 1);
    pub_path_target= n_.advertise<nav_msgs::Path>("/path_target", 1,true);
    //Topic you want to subscribe
    sub_ = n_.subscribe("/odom", 1, &SubscribeAndPublish::callback, this);
    //sub_tra=n_.subscribe("/trajectory_node_list",1,&SubscribeAndPublish::callback_tra,this);
    shape = visualization_msgs::Marker::CUBE;

    last_angle_l=0;
    last_angle_r=0;
    num_=0;
    target_x=tar_x[num_];
    target_y=tar_y[num_];
    count_init=0;

//目标轨迹、姿态
 tar_theta[0]=1.219285;
 tar_theta[1]=1.764561;
 tar_theta[2]=1.678792;
 tar_theta[3]=1.363870;
 tar_theta[4]=1.345865;
 tar_theta[5]=-0.491350;
 tar_theta[6]=0.131505;
 tar_theta[7]=0.987675;
 tar_theta[8]=2.463286;
 tar_theta[9]=0.848399;
 tar_theta[10]=2.350669;
 tar_theta[11]=2.282005;
 tar_theta[12]=2.273913;
 tar_theta[13]=5.086359;
 tar_theta[14]=3.533059;
 tar_theta[15]=3.594498;
 tar_theta[16]=4.213224;
 tar_theta[17]=3.996533;
 tar_theta[18]=2.477230;
 tar_theta[19]=4.556255;
 tar_theta[20]=2.041047;
 tar_theta[21]=3.880281;
 tar_theta[22]=4.177966;
 tar_theta[23]=5.325437;
 tar_theta[24]=3.622182;
 tar_theta[25]=5.186789;
 tar_theta[26]=5.584433;
 tar_theta[27]=6.707058;
 tar_theta[28]=6.016465;
 tar_theta[29]=7.222261;
 tar_theta[30]=10.117354;
 tar_theta[31]=7.056068;
 tar_theta[32]=7.612714;
 tar_theta[33]=7.188706;
 tar_theta[34]=6.087970;
 tar_theta[35]=12.919596;
 tar_theta[36]=11.435529;
 tar_theta[37]=6.212006;
 tar_theta[38]=12.270084;
 tar_theta[39]=13.718867;
 tar_theta[40]=14.383102;
 tar_theta[41]=12.557953;
 tar_theta[42]=7.614700;
 tar_theta[43]=7.423694;
 tar_theta[44]=9.687645;
 tar_theta[45]=10.762102;
 tar_theta[46]=9.376029;
 tar_theta[47]=7.358903;
 tar_theta[48]=7.381254;
 tar_theta[49]=8.250332;
 tar_theta[50]=7.786314;
 tar_theta[51]=7.225861;
 tar_theta[52]=6.945139;
 tar_theta[53]=8.742701;
 tar_theta[54]=8.387884;
 tar_theta[55]=7.424447;
 tar_theta[56]=6.560495;
 tar_theta[57]=2.563393;
 tar_theta[58]=1.830800;
 tar_theta[59]=-0.251846;
 tar_theta[60]=-3.606444;
 tar_theta[61]=-7.634949;
 tar_theta[62]=-11.169566;
 tar_theta[63]=-15.782098;
 tar_theta[64]=-18.716781;
 tar_theta[65]=-21.948267;
 tar_theta[66]=-25.441814;
 tar_theta[67]=-28.733779;
 tar_theta[68]=-32.104568;
 tar_theta[69]=-35.581160;
 tar_theta[70]=-38.375212;
 tar_theta[71]=-42.226515;
 tar_theta[72]=-44.603858;
 tar_theta[73]=-46.819260;
 tar_theta[74]=-52.285777;
 tar_theta[75]=-56.279420;
 tar_theta[76]=-56.062681;
 tar_theta[77]=-62.263654;
 tar_theta[78]=-61.280872;
 tar_theta[79]=-63.371588;
 tar_theta[80]=-63.374652;
 tar_theta[81]=-62.449525;
 tar_theta[82]=-61.242215;
 tar_theta[83]=-62.743252;
 tar_theta[84]=-63.248133;
 tar_theta[85]=-59.039953;
 tar_theta[86]=-55.794017;
 tar_theta[87]=-53.125457;
 tar_theta[88]=-50.894904;
 tar_theta[89]=-48.366549;
 tar_theta[90]=-46.329535;
 tar_theta[91]=-44.462142;
 tar_theta[92]=-41.469401;
 tar_theta[93]=-39.248583;
 tar_theta[94]=-36.749779;
 tar_theta[95]=-34.052585;
 tar_theta[96]=-31.469962;
 tar_theta[97]=-28.656253;
 tar_theta[98]=-27.148794;
 tar_theta[99]=-22.216852;
 tar_theta[100]=-22.385829;
 tar_theta[101]=-18.885081;
 tar_theta[102]=-14.550521;
 tar_theta[103]=-10.975598;
 tar_theta[104]=-9.422765;
 tar_theta[105]=-5.450722;
 tar_theta[106]=-3.378114;
 tar_theta[107]=2.146086;
 tar_theta[108]=3.716312;
 tar_theta[109]=8.041707;
 tar_theta[110]=9.614803;
 tar_theta[111]=14.380550;
 tar_theta[112]=17.777674;
 tar_theta[113]=22.114343;
 tar_theta[114]=26.438891;
 tar_theta[115]=29.941498;
 tar_theta[116]=33.334662;
 tar_theta[117]=36.048306;
 tar_theta[118]=40.784033;
 tar_theta[119]=48.532509;
 tar_theta[120]=51.383101;
 tar_theta[121]=57.498061;
 tar_theta[122]=60.385036;
 tar_theta[123]=64.150982;
 tar_theta[124]=72.059433;
 tar_theta[125]=74.290311;
 tar_theta[126]=79.925859;
 tar_theta[127]=83.947599;
 tar_theta[128]=88.189753;
 tar_theta[129]=91.897132;
 tar_theta[130]=94.695610;
 tar_theta[131]=97.644075;
 tar_theta[132]=96.352791;
 tar_theta[133]=99.430648;
 tar_theta[134]=101.941099;
 tar_theta[135]=97.873776;
 tar_theta[136]=102.376202;
 tar_theta[137]=107.801944;
 tar_theta[138]=110.448556;
 tar_theta[139]=116.162174;
 tar_theta[140]=116.368805;
 tar_theta[141]=121.563049;
 tar_theta[142]=121.208335;
 tar_theta[143]=127.879640;
 tar_theta[144]=131.038766;
 tar_theta[145]=138.292660;
 tar_theta[146]=141.315691;
 tar_theta[147]=145.675650;
 tar_theta[148]=149.497142;
 tar_theta[149]=154.708061;
 tar_theta[150]=157.577202;
 tar_theta[151]=162.837402;
 tar_theta[152]=166.438230;
 tar_theta[153]=171.340345;
 tar_theta[154]=176.677943;
 tar_theta[155]=177.783693;
 tar_theta[156]=177.467755;
 tar_theta[157]=179.942769;
 tar_theta[158]=179.394221;
 tar_theta[159]=178.452096;
 tar_theta[160]=178.312266;
 tar_theta[161]=-178.966291;
 tar_theta[162]=-176.168537;
 tar_theta[163]=-178.367794;
 tar_theta[164]=-178.617659;
 tar_theta[165]=-177.931231;
 tar_theta[166]=-178.238338;
 tar_theta[167]=-178.257519;
 tar_theta[168]=-179.474128;
 tar_theta[169]=-177.207691;
 tar_theta[170]=-176.830333;
 tar_theta[171]=-178.606539;
 tar_theta[172]=-176.446761;
 tar_theta[173]=-178.205005;
 tar_theta[174]=-177.295673;
 tar_theta[175]=-178.787743;
 tar_theta[176]=-177.782835;
 tar_theta[177]=-178.323830;
 tar_theta[178]=-178.762238;
 tar_theta[179]=178.307944;
 tar_theta[180]=179.021577;
 tar_theta[181]=177.998910;
 tar_theta[182]=176.779863;
 tar_theta[183]=175.543032;
 tar_theta[184]=176.139455;
 tar_theta[185]=175.961890;
 tar_theta[186]=174.219491;
 tar_theta[187]=173.132234;
 tar_theta[188]=173.574829;
 tar_theta[189]=173.237586;
 tar_theta[190]=173.970159;
 tar_theta[191]=174.888789;
 tar_theta[192]=172.559362;
 tar_theta[193]=171.253734;
 tar_theta[194]=172.911079;
 tar_theta[195]=173.728347;
 tar_theta[196]=174.436777;
 tar_theta[197]=175.684979;
 tar_theta[198]=173.760271;
 tar_theta[199]=173.544309;
 tar_theta[200]=173.895724;
 tar_theta[201]=170.154594;
 tar_theta[202]=174.423933;
 tar_theta[203]=179.288733;
 tar_theta[204]=-179.208678;
 tar_theta[205]=-179.718797;
 tar_theta[206]=179.351043;
 tar_theta[207]=-178.318186;
 tar_theta[208]=-177.221589;
 tar_theta[209]=-177.128128;
 tar_theta[210]=-178.270336;
 tar_theta[211]=-177.272577;
 tar_theta[212]=-177.352002;
 tar_theta[213]=-177.530823;
 tar_theta[214]=-178.430998;
 tar_theta[215]=-176.659536;
 tar_theta[216]=-177.564540;
 tar_theta[217]=-176.878669;
 tar_theta[218]=-177.863579;
 tar_theta[219]=-176.533882;
 tar_theta[220]=-176.416848;
 tar_theta[221]=-177.999810;
 tar_theta[222]=-175.897711;
 tar_theta[223]=-178.016677;
 tar_theta[224]=-176.118062;
 tar_theta[225]=-175.715932;
 tar_theta[226]=-176.668133;
 tar_theta[227]=-179.642165;
 tar_theta[228]=-177.852682;
 tar_theta[229]=-175.147646;
 tar_theta[230]=-176.506410;
 tar_theta[231]=-177.675600;
 tar_theta[232]=-175.488979;
 tar_theta[233]=-178.811923;
 tar_theta[234]=-178.354197;
 tar_theta[235]=-178.261623;
 tar_theta[236]=-176.795495;
 tar_theta[237]=-177.391244;
 tar_theta[238]=-178.263384;
 tar_theta[239]=-177.864934;
 tar_theta[240]=-177.700614;
 tar_theta[241]=-175.894601;
 tar_theta[242]=-176.506015;
 tar_theta[243]=-178.923018;
 tar_theta[244]=-177.455612;
 tar_theta[245]=-176.755119;
 tar_theta[246]=-175.214874;
 tar_theta[247]=-178.655179;
 tar_theta[248]=-177.975314;
 tar_theta[249]=-178.792305;
 tar_theta[250]=-177.973119;
 tar_theta[251]=-177.841459;
 tar_theta[252]=-175.684821;
 tar_theta[253]=-176.734019;
 tar_theta[254]=-177.821564;
 tar_theta[255]=-178.932092;
 tar_theta[256]=-177.009288;
 tar_theta[257]=-177.009288;

 tar_x[0]=0.001586;
 tar_y[0]=-0.003883;
 tar_x[1]=0.391414;
 tar_y[1]=0.004414;
 tar_x[2]=0.592731;
 tar_y[2]=0.010616;
 tar_x[3]=0.796458;
 tar_y[3]=0.016587;
 tar_x[4]=1.010709;
 tar_y[4]=0.021688;
 tar_x[5]=1.215952;
 tar_y[5]=0.026510;
 tar_x[6]=1.422228;
 tar_y[6]=0.024741;
 tar_x[7]=1.632667;
 tar_y[7]=0.025224;
 tar_x[8]=1.839977;
 tar_y[8]=0.028798;
 tar_x[9]=2.049559;
 tar_y[9]=0.037814;
 tar_x[10]=2.255050;
 tar_y[10]=0.040857;
 tar_x[11]=2.601774;
 tar_y[11]=0.055090;
 tar_x[12]=2.810258;
 tar_y[12]=0.063398;
 tar_x[13]=3.022532;
 tar_y[13]=0.071827;
 tar_x[14]=3.224818;
 tar_y[14]=0.089832;
 tar_x[15]=3.431226;
 tar_y[15]=0.102576;
 tar_x[16]=3.638013;
 tar_y[16]=0.115566;
 tar_x[17]=3.851703;
 tar_y[17]=0.131308;
 tar_x[18]=4.066872;
 tar_y[18]=0.146341;
 tar_x[19]=4.278532;
 tar_y[19]=0.155498;
 tar_x[20]=4.479110;
 tar_y[20]=0.171482;
 tar_x[21]=4.688072;
 tar_y[21]=0.178929;
 tar_x[22]=4.889554;
 tar_y[22]=0.192595;
 tar_x[23]=5.121139;
 tar_y[23]=0.209512;
 tar_x[24]=5.325398;
 tar_y[24]=0.228552;
 tar_x[25]=5.536856;
 tar_y[25]=0.241938;
 tar_x[26]=5.750539;
 tar_y[26]=0.261335;
 tar_x[27]=5.952837;
 tar_y[27]=0.281115;
 tar_x[28]=6.152611;
 tar_y[28]=0.304608;
 tar_x[29]=6.488083;
 tar_y[29]=0.339965;
 tar_x[30]=6.689852;
 tar_y[30]=0.365534;
 tar_x[31]=6.897715;
 tar_y[31]=0.402625;
 tar_x[32]=7.098308;
 tar_y[32]=0.427454;
 tar_x[33]=7.314298;
 tar_y[33]=0.456322;
 tar_x[34]=7.521649;
 tar_y[34]=0.482475;
 tar_x[35]=7.732906;
 tar_y[35]=0.505007;
 tar_x[36]=7.935944;
 tar_y[36]=0.551582;
 tar_x[37]=8.280742;
 tar_y[37]=0.621328;
 tar_x[38]=8.486003;
 tar_y[38]=0.643670;
 tar_x[39]=8.783858;
 tar_y[39]=0.708450;
 tar_x[40]=8.984372;
 tar_y[40]=0.757400;
 tar_x[41]=9.186172;
 tar_y[41]=0.809150;
 tar_x[42]=9.397008;
 tar_y[42]=0.856115;
 tar_x[43]=9.599462;
 tar_y[43]=0.883181;
 tar_x[44]=9.803648;
 tar_y[44]=0.909786;
 tar_x[45]=10.009481;
 tar_y[45]=0.944924;
 tar_x[46]=10.212217;
 tar_y[46]=0.983459;
 tar_x[47]=10.425457;
 tar_y[47]=1.018669;
 tar_x[48]=10.767127;
 tar_y[48]=1.062795;
 tar_x[49]=10.983261;
 tar_y[49]=1.090794;
 tar_x[50]=11.185421;
 tar_y[50]=1.120107;
 tar_x[51]=11.386833;
 tar_y[51]=1.147648;
 tar_x[52]=11.587270;
 tar_y[52]=1.173061;
 tar_x[53]=11.799834;
 tar_y[53]=1.198954;
 tar_x[54]=12.007560;
 tar_y[54]=1.230899;
 tar_x[55]=12.218179;
 tar_y[55]=1.261955;
 tar_x[56]=12.432466;
 tar_y[56]=1.289879;
 tar_x[57]=12.683611;
 tar_y[57]=1.318762;
 tar_x[58]=12.939209;
 tar_y[58]=1.330205;
 tar_x[59]=13.211825;
 tar_y[59]=1.338919;
 tar_x[60]=13.476637;
 tar_y[60]=1.337755;
 tar_x[61]=13.724465;
 tar_y[61]=1.322135;
 tar_x[62]=13.981527;
 tar_y[62]=1.287676;
 tar_x[63]=14.240302;
 tar_y[63]=1.236580;
 tar_x[64]=14.448727;
 tar_y[64]=1.177672;
 tar_x[65]=14.646981;
 tar_y[65]=1.110502;
 tar_x[66]=14.853075;
 tar_y[66]=1.027451;
 tar_x[67]=15.054431;
 tar_y[67]=0.931660;
 tar_x[68]=15.249517;
 tar_y[68]=0.824704;
 tar_x[69]=15.438113;
 tar_y[69]=0.706377;
 tar_x[70]=15.627597;
 tar_y[70]=0.570814;
 tar_x[71]=15.808791;
 tar_y[71]=0.427329;
 tar_x[72]=15.967721;
 tar_y[72]=0.283086;
 tar_x[73]=16.144664;
 tar_y[73]=0.108573;
 tar_x[74]=16.300099;
 tar_y[74]=-0.057060;
 tar_x[75]=16.433206;
 tar_y[75]=-0.229192;
 tar_x[76]=16.558973;
 tar_y[76]=-0.417625;
 tar_x[77]=16.713012;
 tar_y[77]=-0.646537;
 tar_x[78]=16.903435;
 tar_y[78]=-1.008681;
 tar_x[79]=17.006940;
 tar_y[79]=-1.197587;
 tar_x[80]=17.104454;
 tar_y[80]=-1.392077;
 tar_x[81]=17.196629;
 tar_y[81]=-1.575943;
 tar_x[82]=17.294136;
 tar_y[82]=-1.762850;
 tar_x[83]=17.394722;
 tar_y[83]=-1.946135;
 tar_x[84]=17.489290;
 tar_y[84]=-2.129697;
 tar_x[85]=17.588875;
 tar_y[85]=-2.327254;
 tar_x[86]=17.708819;
 tar_y[86]=-2.527190;
 tar_x[87]=17.860437;
 tar_y[87]=-2.750239;
 tar_x[88]=17.981794;
 tar_y[88]=-2.912021;
 tar_x[89]=18.119698;
 tar_y[89]=-3.081681;
 tar_x[90]=18.263750;
 tar_y[90]=-3.243740;
 tar_x[91]=18.412948;
 tar_y[91]=-3.400028;
 tar_x[92]=18.585038;
 tar_y[92]=-3.568917;
 tar_x[93]=18.764240;
 tar_y[93]=-3.727291;
 tar_x[94]=19.001914;
 tar_y[94]=-3.921469;
 tar_x[95]=19.168092;
 tar_y[95]=-4.045559;
 tar_x[96]=19.408776;
 tar_y[96]=-4.208224;
 tar_x[97]=19.583513;
 tar_y[97]=-4.315177;
 tar_x[98]=19.847967;
 tar_y[98]=-4.459699;
 tar_x[99]=20.026157;
 tar_y[99]=-4.551075;
 tar_x[100]=20.221899;
 tar_y[100]=-4.631023;
 tar_x[101]=20.418264;
 tar_y[101]=-4.711902;
 tar_x[102]=20.699980;
 tar_y[102]=-4.808273;
 tar_x[103]=20.927081;
 tar_y[103]=-4.867219;
 tar_x[104]=21.166389;
 tar_y[104]=-4.913630;
 tar_x[105]=21.398479;
 tar_y[105]=-4.952147;
 tar_x[106]=21.641790;
 tar_y[106]=-4.975364;
 tar_x[107]=21.870344;
 tar_y[107]=-4.988855;
 tar_x[108]=22.106429;
 tar_y[108]=-4.980008;
 tar_x[109]=22.343970;
 tar_y[109]=-4.964579;
 tar_x[110]=22.584586;
 tar_y[110]=-4.930584;
 tar_x[111]=22.837226;
 tar_y[111]=-4.887786;
 tar_x[112]=23.055967;
 tar_y[112]=-4.831702;
 tar_x[113]=23.281510;
 tar_y[113]=-4.759385;
 tar_x[114]=23.512267;
 tar_y[114]=-4.665617;
 tar_x[115]=23.720496;
 tar_y[115]=-4.562075;
 tar_x[116]=23.925491;
 tar_y[116]=-4.444000;
 tar_x[117]=24.096956;
 tar_y[117]=-4.331220;
 tar_x[118]=24.281305;
 tar_y[118]=-4.197045;
 tar_x[119]=24.484037;
 tar_y[119]=-4.022150;
 tar_x[120]=24.629180;
 tar_y[120]=-3.857908;
 tar_x[121]=24.759852;
 tar_y[121]=-3.694317;
 tar_x[122]=24.868364;
 tar_y[122]=-3.524000;
 tar_x[123]=24.973410;
 tar_y[123]=-3.339198;
 tar_x[124]=25.065576;
 tar_y[124]=-3.148959;
 tar_x[125]=25.129890;
 tar_y[125]=-2.950320;
 tar_x[126]=25.187995;
 tar_y[126]=-2.743739;
 tar_x[127]=25.226035;
 tar_y[127]=-2.529624;
 tar_x[128]=25.247691;
 tar_y[128]=-2.325378;
 tar_x[129]=25.254468;
 tar_y[129]=-2.110952;
 tar_x[130]=25.245226;
 tar_y[130]=-1.831934;
 tar_x[131]=25.227660;
 tar_y[131]=-1.618074;
 tar_x[132]=25.200742;
 tar_y[132]=-1.417510;
 tar_x[133]=25.178071;
 tar_y[133]=-1.213879;
 tar_x[134]=25.142792;
 tar_y[134]=-1.001481;
 tar_x[135]=25.094743;
 tar_y[135]=-0.774280;
 tar_x[136]=25.067058;
 tar_y[136]=-0.574092;
 tar_x[137]=25.014767;
 tar_y[137]=-0.335787;
 tar_x[138]=24.949668;
 tar_y[138]=-0.133051;
 tar_x[139]=24.872357;
 tar_y[139]=0.074294;
 tar_x[140]=24.776226;
 tar_y[140]=0.269984;
 tar_x[141]=24.680018;
 tar_y[141]=0.464059;
 tar_x[142]=24.570273;
 tar_y[142]=0.642705;
 tar_x[143]=24.457319;
 tar_y[143]=0.829153;
 tar_x[144]=24.302271;
 tar_y[144]=1.028467;
 tar_x[145]=24.142833;
 tar_y[145]=1.211629;
 tar_x[146]=23.969363;
 tar_y[146]=1.366225;
 tar_x[147]=23.776070;
 tar_y[147]=1.520995;
 tar_x[148]=23.570773;
 tar_y[148]=1.661167;
 tar_x[149]=23.373084;
 tar_y[149]=1.777628;
 tar_x[150]=23.148565;
 tar_y[150]=1.883719;
 tar_x[151]=22.915466;
 tar_y[151]=1.979904;
 tar_x[152]=22.682048;
 tar_y[152]=2.051992;
 tar_x[153]=22.485737;
 tar_y[153]=2.099346;
 tar_x[154]=22.235535;
 tar_y[154]=2.137452;
 tar_x[155]=21.968436;
 tar_y[155]=2.152956;
 tar_x[156]=21.599274;
 tar_y[156]=2.167243;
 tar_x[157]=21.388216;
 tar_y[157]=2.176577;
 tar_x[158]=21.174975;
 tar_y[158]=2.176790;
 tar_x[159]=20.971726;
 tar_y[159]=2.178939;
 tar_x[160]=20.764233;
 tar_y[160]=2.184546;
 tar_x[161]=20.515769;
 tar_y[161]=2.191867;
 tar_x[162]=20.304004;
 tar_y[162]=2.188046;
 tar_x[163]=20.089898;
 tar_y[163]=2.173707;
 tar_x[164]=19.888214;
 tar_y[164]=2.167960;
 tar_x[165]=19.682007;
 tar_y[165]=2.162984;
 tar_x[166]=19.481412;
 tar_y[166]=2.155738;
 tar_x[167]=19.264092;
 tar_y[167]=2.149054;
 tar_x[168]=19.064167;
 tar_y[168]=2.142972;
 tar_x[169]=18.862390;
 tar_y[169]=2.141120;
 tar_x[170]=18.656910;
 tar_y[170]=2.131098;
 tar_x[171]=18.454897;
 tar_y[171]=2.119911;
 tar_x[172]=18.246924;
 tar_y[172]=2.114852;
 tar_x[173]=18.041498;
 tar_y[173]=2.102096;
 tar_x[174]=17.828280;
 tar_y[174]=2.095414;
 tar_x[175]=17.608483;
 tar_y[175]=2.085032;
 tar_x[176]=17.392331;
 tar_y[176]=2.080458;
 tar_x[177]=17.180404;
 tar_y[177]=2.072253;
 tar_x[178]=16.971369;
 tar_y[178]=2.066136;
 tar_x[179]=16.722924;
 tar_y[179]=2.060768;
 tar_x[180]=16.417987;
 tar_y[180]=2.069776;
 tar_x[181]=16.215977;
 tar_y[181]=2.073226;
 tar_x[182]=16.001408;
 tar_y[182]=2.080723;
 tar_x[183]=15.656588;
 tar_y[183]=2.100123;
 tar_x[184]=15.444685;
 tar_y[184]=2.116640;
 tar_x[185]=15.236346;
 tar_y[185]=2.130699;
 tar_x[186]=15.024887;
 tar_y[186]=2.145627;
 tar_x[187]=14.718059;
 tar_y[187]=2.176688;
 tar_x[188]=14.518196;
 tar_y[188]=2.200760;
 tar_x[189]=14.317908;
 tar_y[189]=2.223315;
 tar_x[190]=14.099924;
 tar_y[190]=2.249163;
 tar_x[191]=13.900806;
 tar_y[191]=2.270196;
 tar_x[192]=13.691202;
 tar_y[192]=2.288944;
 tar_x[193]=13.491644;
 tar_y[193]=2.315006;
 tar_x[194]=13.283731;
 tar_y[194]=2.346993;
 tar_x[195]=13.069595;
 tar_y[195]=2.373623;
 tar_x[196]=12.865328;
 tar_y[196]=2.396072;
 tar_x[197]=12.657100;
 tar_y[197]=2.416354;
 tar_x[198]=12.449954;
 tar_y[198]=2.431984;
 tar_x[199]=12.241908;
 tar_y[199]=2.454731;
 tar_x[200]=12.029124;
 tar_y[200]=2.478808;
 tar_x[201]=11.823700;
 tar_y[201]=2.500777;
 tar_x[202]=11.611647;
 tar_y[202]=2.537578;
 tar_x[203]=11.359755;
 tar_y[203]=2.562170;
 tar_x[204]=10.986001;
 tar_y[204]=2.566810;
 tar_x[205]=10.774519;
 tar_y[205]=2.563889;
 tar_x[206]=10.560173;
 tar_y[206]=2.562837;
 tar_x[207]=10.350319;
 tar_y[207]=2.565214;
 tar_x[208]=10.142088;
 tar_y[208]=2.559100;
 tar_x[209]=9.934301;
 tar_y[209]=2.549016;
 tar_x[210]=9.580812;
 tar_y[210]=2.531283;
 tar_x[211]=9.377219;
 tar_y[211]=2.525135;
 tar_x[212]=9.166780;
 tar_y[212]=2.515110;
 tar_x[213]=8.956571;
 tar_y[213]=2.505388;
 tar_x[214]=8.744243;
 tar_y[214]=2.496232;
 tar_x[215]=8.530889;
 tar_y[215]=2.490388;
 tar_x[216]=8.329461;
 tar_y[216]=2.478631;
 tar_x[217]=8.117505;
 tar_y[217]=2.469616;
 tar_x[218]=7.916190;
 tar_y[218]=2.458638;
 tar_x[219]=7.701580;
 tar_y[219]=2.450632;
 tar_x[220]=7.485232;
 tar_y[220]=2.437528;
 tar_x[221]=7.285358;
 tar_y[221]=2.425012;
 tar_x[222]=7.072210;
 tar_y[222]=2.417568;
 tar_x[223]=6.860473;
 tar_y[223]=2.402382;
 tar_x[224]=6.645944;
 tar_y[224]=2.394953;
 tar_x[225]=6.432685;
 tar_y[225]=2.380482;
 tar_x[226]=6.225425;
 tar_y[226]=2.364956;
 tar_x[227]=6.017447;
 tar_y[227]=2.352848;
 tar_x[228]=5.800010;
 tar_y[228]=2.351490;
 tar_x[229]=5.596518;
 tar_y[229]=2.343860;
 tar_x[230]=5.385310;
 tar_y[230]=2.325930;
 tar_x[231]=5.173796;
 tar_y[231]=2.313017;
 tar_x[232]=4.963945;
 tar_y[232]=2.304499;
 tar_x[233]=4.745326;
 tar_y[233]=2.287251;
 tar_x[234]=4.534562;
 tar_y[234]=2.282880;
 tar_x[235]=4.323582;
 tar_y[235]=2.276818;
 tar_x[236]=4.122328;
 tar_y[236]=2.270710;
 tar_x[237]=3.910263;
 tar_y[237]=2.258837;
 tar_x[238]=3.707576;
 tar_y[238]=2.249602;
 tar_x[239]=3.490550;
 tar_y[239]=2.243022;
 tar_x[240]=3.281973;
 tar_y[240]=2.235246;
 tar_x[241]=3.080944;
 tar_y[241]=2.227174;
 tar_x[242]=2.875526;
 tar_y[242]=2.212430;
 tar_x[243]=2.666853;
 tar_y[243]=2.199689;
 tar_x[244]=2.451364;
 tar_y[244]=2.195638;
 tar_x[245]=2.243925;
 tar_y[245]=2.186420;
 tar_x[246]=2.041736;
 tar_y[246]=2.174957;
 tar_x[247]=1.841559;
 tar_y[247]=2.158200;
 tar_x[248]=1.626274;
 tar_y[248]=2.153146;
 tar_x[249]=1.414632;
 tar_y[249]=2.145664;
 tar_x[250]=1.209617;
 tar_y[250]=2.141342;
 tar_x[251]=1.000917;
 tar_y[251]=2.133956;
 tar_x[252]=0.797237;
 tar_y[252]=2.126279;
 tar_x[253]=0.585659;
 tar_y[253]=2.110314;
 tar_x[254]=0.376052;
 tar_y[254]=2.098353;
 tar_x[255]=0.171868;
 tar_y[255]=2.090586;
 tar_x[256]=-0.041429;
 tar_y[256]=2.086610;
 tar_x[257]=-0.246175;
 tar_y[257]=2.075913;

for(count_path=0;count_path<258;count_path++)
 {
   pose_target.pose.position.x=tar_x[count_path];
   pose_target.pose.position.y=tar_y[count_path];
   pose_target.pose.position.z=0;
   pose_target.pose.orientation.x=0;
   pose_target.pose.orientation.y=0;
   pose_target.pose.orientation.z=0;
   pose_target.pose.orientation.w=1;
   pose_target.header.stamp=current_time_target;
   pose_target.header.frame_id="map";
   msg_target.poses.push_back(pose_target);
   msg_target.header.frame_id="map";
   msg_target.header.stamp=current_time_target;
 }
}//构造函数结尾
 
  void callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
  array.markers.clear();
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker2.header.frame_id = "map";
  marker2.header.stamp = ros::Time::now();
  marker3.header.frame_id = "map";
  marker3.header.stamp = ros::Time::now();
  marker.ns = "b";
  marker2.ns = "b";
  marker2.id = 0;
  marker.id = 1;
  marker3.ns = "b";
  marker3.id = 2;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker2.type = visualization_msgs::Marker::CUBE;
  marker3.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker2.action = visualization_msgs::Marker::ADD;
  marker3.action = visualization_msgs::Marker::ADD;
  current_time=ros::Time::now();

  //获取当前AGV位置姿态
  tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  x=msg->pose.pose.position.x;
  y=msg->pose.pose.position.y;
  z=msg->pose.pose.position.z;

  //目标点初始化
  if(count_init<1)
  {
  d_min_init_yaw(x,y,yaw);
  num_=min_ind+10;
  if(num_>257)
  {
    num_=257;
  }
  target_x=tar_x[num_];
  target_y=tar_y[num_];
  if(d_min_init<1)
    count++;

  }

  //计算AGV距离轨迹的最新间距，更新预瞄距离
  d_min_refersh(x,y);
  if(d_min<0.5)
  {
    d_ym=1;
  }
  else
  {
    d_ym=d_min+1;
  }

 //计算目标姿态角度
    if(target_y-y<0.01 && target_x-x>0)
  {
     target_angle=0;
  }
  if(target_x-x>0 && target_y-y>0)
  {
    target_angle=atan((target_y-y)/(target_x-x))*57.29578;
  }
    if(target_x-x>0 && target_y-y<0)
  {
    target_angle=atan((target_y-y)/(target_x-x))*57.29578;
  }
  if(target_x-x<0 && target_y-y>0)
  {
    target_angle=180+atan((target_y-y)/(target_x-x))*57.29578;
  }
   if(target_x-x<0 && target_y-y<0)
  {
    target_angle=-180+atan((target_y-y)/(target_x-x))*57.29578;
  }

  //计算当前姿态与目标姿态的差值
  theta=target_angle-yaw*57.29578; 
  abs_theta=fabs(theta);
  if(theta>180)
  {
    theta=theta-360;
  }
  if(theta<-180)
  {
    theta=theta+360;
  }
  if(theta==180||theta==-180)
  {
    ROS_INFO("the angle of agv is inverse!!!");
  }
  
  //计算预瞄距离
  d=sqrt((target_x-x)*(target_x-x)+(target_y-y)*(target_y-y));
    while (d<d_ym && ros::ok())
  {
     num_=num_+1;
     if(num_>257)
     {
        num_=257;
        break;
     }
      target_x=tar_x[num_];
      target_y=tar_y[num_];
     d=sqrt((target_x-x)*(target_x-x)+(target_y-y)*(target_y-y));
  }
  
//计算角速度
if(theta<40 && theta>20)
 {
    msg_twist.angular.z=3*theta/400;
 }
 if(theta<20&&theta>0)
 {
   msg_twist.angular.z=3*theta/400;
 }
 if(theta==40 || theta>40)
 {
   msg_twist.angular.z=0.3;
 }
 if(theta<0 && theta>-20)
 {
   msg_twist.angular.z=3*theta/400;
 }
 if(theta<-20 && theta>-40)
 {
    msg_twist.angular.z=3*theta/400;
 }
 if(theta==20)
 {
   msg_twist.angular.z=0.15;
 }
 if(theta==-20)
 {
   msg_twist.angular.z=-0.15;
 }
 if(theta<-40 || theta==-40)
 {
    msg_twist.angular.z=-0.3;
 }

 //设定线速度
  msg_twist.linear.x=0.4;

  //轨迹终点停车
      if(num_==257 && d<0.3)
     {
       msg_twist.linear.x=0;
        msg_twist.linear.y=0;
        msg_twist.linear.z=0;
        msg_twist.angular.x=0;
        msg_twist.angular.y=0;
        msg_twist.angular.z=0;
        ROS_INFO("arrive");     
     }
  
  //计算AGV前轮转角
 if(msg_twist.linear.x!=0)
 {
 r=fabs(msg_twist.linear.x/msg_twist.angular.z);
 theta2=atan(1.42/(2*r-1.16))*57.29578;
 theta1=atan(1.42/(2*r+1.16))*57.29578;
 if(msg_twist.angular.z<0)
 {
   theta1=-theta1;
   theta2=-theta2;
 }
 if(msg_twist.angular.z>0)
 {
    theta_temp=theta1;
    theta1=theta2;
    theta2=theta_temp;
 }
  if(msg_twist.angular.z==0)
 {
    theta1=0;
    theta2=0;
 }

 //计算AGV前轮转速
 v2=fabs(msg_twist.angular.z*sqrt(pow(r-0.58,2)+pow(0.71,2)));
 v1=fabs(msg_twist.angular.z*sqrt(pow(r+0.58,2)+pow(0.71,2)));
 }
 else
 {
   theta1=0;
   theta2=0;
   v1=0;
   v2=0;
 }

 //100ms内前轮转角变化量
 angle_delta_l=fabs(theta1-last_angle_l);
 angle_delta_r=fabs(theta2-last_angle_r);
 last_angle_l=theta1;
 last_angle_r=theta2;

 //将转速转角存入buffer中
 int_theta1=round(theta1*100)+6000;
 int_theta2=round(theta2*100)+6000;
 int_v1=round(v1*1000);
 int_v2=round(v2*1000);
 buffer[0]=int_theta1/256;
 buffer[1]=int_theta1%256;
 buffer[2]=int_theta2/256;
 buffer[3]=int_theta2%256;
 buffer[4]=int_v1/256;
 buffer[5]=int_v1%256; 
 buffer[6]=int_v2/256;
 buffer[7]=int_v2%256;

 //发布AGV实时轨迹
 pose_sta.pose.position.x=msg->pose.pose.position.x;
 pose_sta.pose.position.y=msg->pose.pose.position.y;
 pose_sta.pose.position.z=msg->pose.pose.position.z;
 pose_sta.pose.orientation.x=msg->pose.pose.orientation.x;
 pose_sta.pose.orientation.y=msg->pose.pose.orientation.y;
 pose_sta.pose.orientation.z=msg->pose.pose.orientation.z;
 pose_sta.pose.orientation.w=msg->pose.pose.orientation.w;
 pose_sta.header.stamp=current_time;
 pose_sta.header.frame_id="map";
 msg_path.poses.push_back(pose_sta);
 msg_path.header.frame_id="map";
 msg_path.header.stamp=current_time;

//发布AGV车身姿态，目标点
    marker2.pose.position.x = x;
    marker2.pose.position.y = y;
    marker2.pose.position.z = 0;
    marker2.pose.orientation.x = msg->pose.pose.orientation.x;
    marker2.pose.orientation.y = msg->pose.pose.orientation.y;
    marker2.pose.orientation.z = msg->pose.pose.orientation.z;
    marker2.pose.orientation.w = msg->pose.pose.orientation.w;
 
    marker.pose.position.x = target_x;
    marker.pose.position.y = target_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;


    marker3.pose.position.x = x;
    marker3.pose.position.y = y;
    marker3.pose.position.z = 0;
    marker3.pose.orientation.x = 0.0;
    marker3.pose.orientation.y = 0.0;
    marker3.pose.orientation.z = 0.0;
    marker3.pose.orientation.w = 1.0;

    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker2.scale.x = 1.7;
    marker2.scale.y = 1.4;
    marker2.scale.z = 0.4;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker2.color.r = 0.0f;
    marker2.color.g = 1.0f;
    marker2.color.b = 0.0f;
    marker2.color.a = 1.0;

    marker3.color.r = 1.0f;
    marker3.color.g = 0.0f;
    marker3.color.b = 0.0f;
    marker3.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker2.lifetime = ros::Duration();
    marker3.lifetime = ros::Duration();
    array.markers.push_back(marker);
    array.markers.push_back(marker2);
    array.markers.push_back(marker3);
 //printf("delta_l:%f\n,delta_r:%f\n",angle_delta_l,angle_delta_r);
 //ROS_INFO("d:%f\n theta1:%f\n theta2:%f\n num:%d\n",d,theta1,theta2,num_);
 //ROS_INFO("theta1:%f,theta3:%f,v1:%f,v3:%f",theta1,theta3,v1,v3);
 //printf(" theta:%f\n delta_angle_l:%f\n delta_angle_r:%f\n d:%f\n num:%d\n,d_ym:%f",theta,angle_delta_l,angle_delta_r,d,num_,d_ym);
 printf("num:%d,d_ym:%f\n",num_,d_ym);
 pub_.publish(msg_twist);
 pub_path.publish(msg_path);
 pub_path_target.publish(msg_target);
 marker_pub.publish(array);
}

void target_init(const double& x_init,const double& y_init)
{
  d_min_init=sqrt((tar_x[0]-x_init)*(tar_x[0]-x_init)+(tar_y[0]-y_init)*(tar_y[0]-y_init));
  min_ind=0;
for(count_d=1;count_d<258;count_d++)
  {
    d_temp=sqrt((tar_x[count_d]-x_init)*(tar_x[count_d]-x_init)+(tar_y[count_d]-y_init)*(tar_y[count_d]-y_init));
    if(d_temp<d_min_init)
    {
      d_min_init=d_temp;
      min_ind=count_d;
    }
  }
    num_=min_ind;
    target_x=tar_x[num_];
    target_y=tar_y[num_];
    return;
}

void d_min_refersh(const double& x_init,const double& y_init)
{
  d_min=sqrt((tar_x[0]-x_init)*(tar_x[0]-x_init)+(tar_y[0]-y_init)*(tar_y[0]-y_init));
  min_ind=0;
for(count_d=1;count_d<258;count_d++)
  {
    d_temp=sqrt((tar_x[count_d]-x_init)*(tar_x[count_d]-x_init)+(tar_y[count_d]-y_init)*(tar_y[count_d]-y_init));
    if(d_temp<d_min)
    {
      d_min=d_temp;
      min_ind=count_d;
    }
  }
    return;
}
void d_min_init_yaw(const double& x_init,const double& y_init,const double& yaw)
{
  if(fabs(yaw*57.29578)<90)
  {
  d_min=sqrt((tar_x[0]-x_init)*(tar_x[0]-x_init)+(tar_y[0]-y_init)*(tar_y[0]-y_init));
  min_ind=0;
  for(count_d=1;count_d<129;count_d++)
  {
    d_temp=sqrt((tar_x[count_d]-x_init)*(tar_x[count_d]-x_init)+(tar_y[count_d]-y_init)*(tar_y[count_d]-y_init));
    if(d_temp<d_min)
    {
      d_min=d_temp;
      min_ind=count_d;
    }
  }
  }
    if(fabs(yaw*57.29578)>90)
    {
       d_min=sqrt((tar_x[129]-x_init)*(tar_x[129]-x_init)+(tar_y[129]-y_init)*(tar_y[129]-y_init));
       min_ind=129;
  for(count_d=129;count_d<258;count_d++)
  {
    d_temp=sqrt((tar_x[count_d]-x_init)*(tar_x[count_d]-x_init)+(tar_y[count_d]-y_init)*(tar_y[count_d]-y_init));
    if(d_temp<d_min)
    {
      d_min=d_temp;
      min_ind=count_d;
    }
  }
    }
    return;
}
 
private:
  tf::Matrix3x3 mat,mat_inve;
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Publisher pub_path;
  ros::Publisher pub_path_target;  
  ros::Subscriber sub_;
  ros::Subscriber sub_tra;
  geometry_msgs::PoseStamped pose_sta;
  geometry_msgs::PoseStamped pose_target;
  geometry_msgs::Twist msg_twist;
  nav_msgs::Path msg_path;
  nav_msgs::Path msg_target;
  ros::Time current_time;
  ros::Time current_time_target;
  int angle;
  double cur_angle;
  double target_x;
  double target_y;
  double tar_x[1000];
  double tar_y[1000];
  int jiaodu;
  int num,min_ind;
  int num_;
  int count;
  int count_tar;
  double r;
  double theta1,theta2,theta3,theta4,theta_temp;
  double v1,v2,v3,v4;
  canHandle h;
  unsigned int flag;
  long id;
  long id_1;
  unsigned char buffer[8];
  long int msg_1,msg_2,msg_3;
  int int_theta1,int_theta2,int_v1,int_v2;
  unsigned int dlc;
  int count_2;
  double d;
  double last_angle_r,last_angle_l,angle_delta_l,angle_delta_r;
  int count_path;
  ros::Publisher marker_pub;
  visualization_msgs::Marker marker;
  visualization_msgs::Marker marker2;
  visualization_msgs::Marker marker3;
  uint32_t shape;
  visualization_msgs::MarkerArray array;
  Eigen::Matrix3d ro;
  Eigen::Vector3d vc;
  tf::Vector3 v_world;
  tf::Vector3 v_mov;
  double d_min;
  int count_d;
  double d_temp;
  double x_init,y_init;
  double x,y,z;
  double d_ym,d_x;
  double abs_theta;
  double tar_theta[1000];
  tf::Quaternion quat;
  geometry_msgs::Quaternion q;
   double roll, pitch, yaw;
   int count_init;
   double delta_angle_l,delta_angle_r;
   double theta,target_angle;
   double d_min_init;
};
 
int main(int argc, char **argv)
{
  //static double d;
  //Initiate ROS
  ros::init(argc, argv, "cmd_vel");
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject=SubscribeAndPublish();
  ROS_INFO("callback\n");
  ros::spin();
  //canBusOff(h);   
  //canClose(h);
  return 0;
}