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
 tar_theta[0]=-4.676203;
 tar_theta[1]=-2.491961;
 tar_theta[2]=0.371182;
 tar_theta[3]=0.557962;
 tar_theta[4]=0.248413;
 tar_theta[5]=-1.165678;
 tar_theta[6]=-1.691749;
 tar_theta[7]=0.554789;
 tar_theta[8]=0.842445;
 tar_theta[9]=0.233635;
 tar_theta[10]=0.321427;
 tar_theta[11]=0.420584;
 tar_theta[12]=-4.267953;
 tar_theta[13]=0.112160;
 tar_theta[14]=1.144009;
 tar_theta[15]=0.862061;
 tar_theta[16]=-1.757748;
 tar_theta[17]=0.483102;
 tar_theta[18]=-1.557195;
 tar_theta[19]=0.715406;
 tar_theta[20]=-1.585620;
 tar_theta[21]=1.521691;
 tar_theta[22]=0.810258;
 tar_theta[23]=0.076680;
 tar_theta[24]=0.900908;
 tar_theta[25]=-0.828843;
 tar_theta[26]=1.932467;
 tar_theta[27]=3.435341;
 tar_theta[28]=-0.214864;
 tar_theta[29]=-2.055478;
 tar_theta[30]=0.899514;
 tar_theta[31]=-1.922712;
 tar_theta[32]=-0.587626;
 tar_theta[33]=-3.607569;
 tar_theta[34]=-3.825158;
 tar_theta[35]=-6.150474;
 tar_theta[36]=-5.613907;
 tar_theta[37]=-10.107203;
 tar_theta[38]=-12.534105;
 tar_theta[39]=-15.341195;
 tar_theta[40]=-15.125696;
 tar_theta[41]=-18.276103;
 tar_theta[42]=-16.038083;
 tar_theta[43]=-18.290938;
 tar_theta[44]=-19.116641;
 tar_theta[45]=-17.926262;
 tar_theta[46]=-19.340510;
 tar_theta[47]=-19.553853;
 tar_theta[48]=-17.668830;
 tar_theta[49]=-17.433022;
 tar_theta[50]=-19.195536;
 tar_theta[51]=-19.455093;
 tar_theta[52]=-21.523115;
 tar_theta[53]=-22.106607;
 tar_theta[54]=-26.350360;
 tar_theta[55]=-29.646785;
 tar_theta[56]=-31.634023;
 tar_theta[57]=-37.831742;
 tar_theta[58]=-42.171362;
 tar_theta[59]=-45.052808;
 tar_theta[60]=-49.056311;
 tar_theta[61]=-51.304989;
 tar_theta[62]=-53.150446;
 tar_theta[63]=-54.975392;
 tar_theta[64]=-56.158621;
 tar_theta[65]=-53.341033;
 tar_theta[66]=-50.364311;
 tar_theta[67]=-53.357783;
 tar_theta[68]=-47.361295;
 tar_theta[69]=-48.629927;
 tar_theta[70]=-52.500405;
 tar_theta[71]=-50.583111;
 tar_theta[72]=-51.225107;
 tar_theta[73]=-47.806812;
 tar_theta[74]=-44.953335;
 tar_theta[75]=-43.718792;
 tar_theta[76]=-43.247435;
 tar_theta[77]=-37.096087;
 tar_theta[78]=-36.446005;
 tar_theta[79]=-34.416108;
 tar_theta[80]=-31.461683;
 tar_theta[81]=-29.318521;
 tar_theta[82]=-28.927241;
 tar_theta[83]=-24.807434;
 tar_theta[84]=-22.928063;
 tar_theta[85]=-19.481447;
 tar_theta[86]=-20.453940;
 tar_theta[87]=-14.835181;
 tar_theta[88]=-14.381773;
 tar_theta[89]=-13.116834;
 tar_theta[90]=-10.922494;
 tar_theta[91]=-6.650667;
 tar_theta[92]=-5.494490;
 tar_theta[93]=-7.175378;
 tar_theta[94]=-1.985761;
 tar_theta[95]=-0.178113;
 tar_theta[96]=3.517531;
 tar_theta[97]=3.874341;
 tar_theta[98]=11.075669;
 tar_theta[99]=12.513151;
 tar_theta[100]=15.388752;
 tar_theta[101]=16.864799;
 tar_theta[102]=20.485744;
 tar_theta[103]=24.307250;
 tar_theta[104]=26.737931;
 tar_theta[105]=30.678645;
 tar_theta[106]=33.340809;
 tar_theta[107]=38.333400;
 tar_theta[108]=38.717481;
 tar_theta[109]=43.485608;
 tar_theta[110]=47.214432;
 tar_theta[111]=51.865838;
 tar_theta[112]=57.121592;
 tar_theta[113]=62.798853;
 tar_theta[114]=66.041389;
 tar_theta[115]=74.301387;
 tar_theta[116]=74.201166;
 tar_theta[117]=82.264121;
 tar_theta[118]=82.830978;
 tar_theta[119]=89.024405;
 tar_theta[120]=96.222414;
 tar_theta[121]=99.446248;
 tar_theta[122]=102.756208;
 tar_theta[123]=109.311429;
 tar_theta[124]=112.115863;
 tar_theta[125]=118.458189;
 tar_theta[126]=123.608640;
 tar_theta[127]=126.524919;
 tar_theta[128]=133.698670;
 tar_theta[129]=135.884252;
 tar_theta[130]=142.409227;
 tar_theta[131]=147.308468;
 tar_theta[132]=152.100319;
 tar_theta[133]=157.060923;
 tar_theta[134]=158.140237;
 tar_theta[135]=162.031493;
 tar_theta[136]=166.500876;
 tar_theta[137]=166.600749;
 tar_theta[138]=172.626114;
 tar_theta[139]=173.824489;
 tar_theta[140]=176.210146;
 tar_theta[141]=177.479316;
 tar_theta[142]=178.534594;
 tar_theta[143]=176.876389;
 tar_theta[144]=178.956432;
 tar_theta[145]=176.681498;
 tar_theta[146]=178.138928;
 tar_theta[147]=177.537984;
 tar_theta[148]=177.361780;
 tar_theta[149]=177.985134;
 tar_theta[150]=178.340291;
 tar_theta[151]=176.857027;
 tar_theta[152]=178.029073;
 tar_theta[153]=178.823891;
 tar_theta[154]=176.936976;
 tar_theta[155]=178.741174;
 tar_theta[156]=177.052852;
 tar_theta[157]=176.650003;
 tar_theta[158]=176.965836;
 tar_theta[159]=178.818204;
 tar_theta[160]=177.242203;
 tar_theta[161]=179.245128;
 tar_theta[162]=176.950514;
 tar_theta[163]=178.158954;
 tar_theta[164]=177.373546;
 tar_theta[165]=179.104661;
 tar_theta[166]=179.306348;
 tar_theta[167]=178.676262;
 tar_theta[168]=175.618144;
 tar_theta[169]=178.461583;
 tar_theta[170]=-179.520422;
 tar_theta[171]=179.923669;
 tar_theta[172]=178.978785;
 tar_theta[173]=177.991111;
 tar_theta[174]=176.690976;
 tar_theta[175]=177.107808;
 tar_theta[176]=179.197652;
 tar_theta[177]=177.763737;
 tar_theta[178]=178.032051;
 tar_theta[179]=179.590209;
 tar_theta[180]=176.997932;
 tar_theta[181]=177.501493;
 tar_theta[182]=177.917152;
 tar_theta[183]=179.375406;
 tar_theta[184]=179.196071;
 tar_theta[185]=176.435339;
 tar_theta[186]=179.639236;
 tar_theta[187]=-179.705162;
 tar_theta[188]=178.593824;
 tar_theta[189]=177.706874;
 tar_theta[190]=-178.118774;
 tar_theta[191]=179.919540;
 tar_theta[192]=178.612477;
 tar_theta[193]=179.727605;
 tar_theta[194]=179.624032;
 tar_theta[195]=177.341184;
 tar_theta[196]=178.769113;
 tar_theta[197]=-179.107603;
 tar_theta[198]=177.906951;
 tar_theta[199]=-178.431871;
 tar_theta[200]=179.241393;
 tar_theta[201]=178.860248;
 tar_theta[202]=178.457201;
 tar_theta[203]=179.542109;
 tar_theta[204]=-179.985765;
 tar_theta[205]=178.480510;
 tar_theta[206]=-179.868168;
 tar_theta[207]=176.723541;
 tar_theta[208]=179.427240;
 tar_theta[209]=179.647245;
 tar_theta[210]=178.224440;
 tar_theta[211]=-179.637068;
 tar_theta[212]=178.999848;
 tar_theta[213]=179.269866;
 tar_theta[214]=178.322578;
 tar_theta[215]=179.921822;
 tar_theta[216]=179.277654;
 tar_theta[217]=178.999224;
 tar_theta[218]=-179.079851;
 tar_theta[219]=177.465256;
 tar_theta[220]=179.627110;
 tar_theta[221]=178.897718;
 tar_theta[222]=179.838262;
 tar_theta[223]=176.373776;
 tar_theta[224]=-178.530294;
 tar_theta[225]=-178.622042;
 tar_theta[226]=175.588238;
 tar_theta[227]=-179.828026;
 tar_theta[228]=-179.735449;
 tar_theta[229]=177.923020;
 tar_theta[230]=179.338103;
 tar_theta[231]=178.798441;
 tar_theta[232]=-178.839895;
 tar_theta[233]=179.407484;
 tar_theta[234]=-177.946930;
 tar_theta[235]=179.959323;
 tar_theta[236]=-178.730730;
 tar_theta[237]=179.916637;
 tar_theta[238]=-178.789392;
 tar_theta[239]=-179.517609;
 tar_theta[240]=-179.517609;
 tar_x[0]=0.000000;
 tar_y[0]=0.000000;
 tar_x[1]=0.199751;
 tar_y[1]=-0.016339;
 tar_x[2]=0.404161;
 tar_y[2]=-0.025235;
 tar_x[3]=0.611001;
 tar_y[3]=-0.023895;
 tar_x[4]=0.826330;
 tar_y[4]=-0.021798;
 tar_x[5]=1.029068;
 tar_y[5]=-0.020919;
 tar_x[6]=1.234349;
 tar_y[6]=-0.025096;
 tar_x[7]=1.447891;
 tar_y[7]=-0.031403;
 tar_x[8]=1.657739;
 tar_y[8]=-0.029371;
 tar_x[9]=1.860738;
 tar_y[9]=-0.026386;
 tar_x[10]=2.067471;
 tar_y[10]=-0.025543;
 tar_x[11]=2.279413;
 tar_y[11]=-0.024354;
 tar_x[12]=2.480075;
 tar_y[12]=-0.022881;
 tar_x[13]=2.692931;
 tar_y[13]=-0.038766;
 tar_x[14]=2.897267;
 tar_y[14]=-0.038366;
 tar_x[15]=3.099026;
 tar_y[15]=-0.034337;
 tar_x[16]=3.305779;
 tar_y[16]=-0.031226;
 tar_x[17]=3.510223;
 tar_y[17]=-0.037500;
 tar_x[18]=3.717412;
 tar_y[18]=-0.035753;
 tar_x[19]=3.926131;
 tar_y[19]=-0.041427;
 tar_x[20]=4.139796;
 tar_y[20]=-0.038759;
 tar_x[21]=4.354706;
 tar_y[21]=-0.044708;
 tar_x[22]=4.556590;
 tar_y[22]=-0.039345;
 tar_x[23]=4.766240;
 tar_y[23]=-0.036380;
 tar_x[24]=4.972468;
 tar_y[24]=-0.036104;
 tar_x[25]=5.178508;
 tar_y[25]=-0.032864;
 tar_x[26]=5.388779;
 tar_y[26]=-0.035906;
 tar_x[27]=5.605046;
 tar_y[27]=-0.028609;
 tar_x[28]=5.822704;
 tar_y[28]=-0.015543;
 tar_x[29]=6.028565;
 tar_y[29]=-0.016315;
 tar_x[30]=6.234359;
 tar_y[30]=-0.023701;
 tar_x[31]=6.437470;
 tar_y[31]=-0.020512;
 tar_x[32]=6.637528;
 tar_y[32]=-0.027228;
 tar_x[33]=6.845204;
 tar_y[33]=-0.029358;
 tar_x[34]=7.048702;
 tar_y[34]=-0.042188;
 tar_x[35]=7.295752;
 tar_y[35]=-0.058706;
 tar_x[36]=7.523786;
 tar_y[36]=-0.083279;
 tar_x[37]=7.731170;
 tar_y[37]=-0.103664;
 tar_x[38]=8.001701;
 tar_y[38]=-0.151888;
 tar_x[39]=8.200982;
 tar_y[39]=-0.196192;
 tar_x[40]=8.406984;
 tar_y[40]=-0.252707;
 tar_x[41]=8.606128;
 tar_y[41]=-0.306536;
 tar_x[42]=8.796753;
 tar_y[42]=-0.369491;
 tar_x[43]=8.991629;
 tar_y[43]=-0.425511;
 tar_x[44]=9.187419;
 tar_y[44]=-0.490228;
 tar_x[45]=9.379940;
 tar_y[45]=-0.556957;
 tar_x[46]=9.574059;
 tar_y[46]=-0.619754;
 tar_x[47]=9.764008;
 tar_y[47]=-0.686424;
 tar_x[48]=9.956225;
 tar_y[48]=-0.754695;
 tar_x[49]=10.333187;
 tar_y[49]=-0.874773;
 tar_x[50]=10.530710;
 tar_y[50]=-0.936798;
 tar_x[51]=10.719738;
 tar_y[51]=-1.002608;
 tar_x[52]=10.990194;
 tar_y[52]=-1.098143;
 tar_x[53]=11.188294;
 tar_y[53]=-1.176269;
 tar_x[54]=11.393261;
 tar_y[54]=-1.259525;
 tar_x[55]=11.605449;
 tar_y[55]=-1.364627;
 tar_x[56]=11.810641;
 tar_y[56]=-1.481414;
 tar_x[57]=11.999595;
 tar_y[57]=-1.597814;
 tar_x[58]=12.192803;
 tar_y[58]=-1.747853;
 tar_x[59]=12.372603;
 tar_y[59]=-1.910722;
 tar_x[60]=12.514606;
 tar_y[60]=-2.052987;
 tar_x[61]=12.670743;
 tar_y[61]=-2.232959;
 tar_x[62]=12.810108;
 tar_y[62]=-2.406946;
 tar_x[63]=13.020230;
 tar_y[63]=-2.687316;
 tar_x[64]=13.136624;
 tar_y[64]=-2.853392;
 tar_x[65]=13.250613;
 tar_y[65]=-3.023401;
 tar_x[66]=13.478540;
 tar_y[66]=-3.329646;
 tar_x[67]=13.608560;
 tar_y[67]=-3.486614;
 tar_x[68]=13.730814;
 tar_y[68]=-3.650976;
 tar_x[69]=13.871621;
 tar_y[69]=-3.803895;
 tar_x[70]=14.008815;
 tar_y[70]=-3.959675;
 tar_x[71]=14.132072;
 tar_y[71]=-4.120309;
 tar_x[72]=14.263911;
 tar_y[72]=-4.280716;
 tar_x[73]=14.484052;
 tar_y[73]=-4.554762;
 tar_x[74]=14.630682;
 tar_y[74]=-4.716511;
 tar_x[75]=14.784898;
 tar_y[75]=-4.870476;
 tar_x[76]=14.942721;
 tar_y[76]=-5.021394;
 tar_x[77]=15.100809;
 tar_y[77]=-5.170095;
 tar_x[78]=15.273195;
 tar_y[78]=-5.300451;
 tar_x[79]=15.518774;
 tar_y[79]=-5.481812;
 tar_x[80]=15.769885;
 tar_y[80]=-5.653855;
 tar_x[81]=15.946617;
 tar_y[81]=-5.761994;
 tar_x[82]=16.122107;
 tar_y[82]=-5.860549;
 tar_x[83]=16.302002;
 tar_y[83]=-5.959968;
 tar_x[84]=16.505482;
 tar_y[84]=-6.054021;
 tar_x[85]=16.700395;
 tar_y[85]=-6.136468;
 tar_x[86]=16.907683;
 tar_y[86]=-6.209797;
 tar_x[87]=17.095369;
 tar_y[87]=-6.279798;
 tar_x[88]=17.304420;
 tar_y[88]=-6.335169;
 tar_x[89]=17.646542;
 tar_y[89]=-6.422895;
 tar_x[90]=18.006211;
 tar_y[90]=-6.506704;
 tar_x[91]=18.308781;
 tar_y[91]=-6.565093;
 tar_x[92]=18.553429;
 tar_y[92]=-6.593619;
 tar_x[93]=18.769185;
 tar_y[93]=-6.614373;
 tar_x[94]=18.978077;
 tar_y[94]=-6.640671;
 tar_x[95]=19.189141;
 tar_y[95]=-6.647989;
 tar_x[96]=19.413996;
 tar_y[96]=-6.648688;
 tar_x[97]=19.648827;
 tar_y[97]=-6.634253;
 tar_x[98]=19.912946;
 tar_y[98]=-6.616366;
 tar_x[99]=20.135458;
 tar_y[99]=-6.572809;
 tar_x[100]=20.377650;
 tar_y[100]=-6.519058;
 tar_x[101]=20.606353;
 tar_y[101]=-6.456111;
 tar_x[102]=20.833206;
 tar_y[102]=-6.387340;
 tar_x[103]=21.041139;
 tar_y[103]=-6.309656;
 tar_x[104]=21.250883;
 tar_y[104]=-6.214921;
 tar_x[105]=21.452611;
 tar_y[105]=-6.113295;
 tar_x[106]=21.657903;
 tar_y[106]=-5.991505;
 tar_x[107]=21.854610;
 tar_y[107]=-5.862092;
 tar_x[108]=22.033974;
 tar_y[108]=-5.720269;
 tar_x[109]=22.214833;
 tar_y[109]=-5.575283;
 tar_x[110]=22.362447;
 tar_y[110]=-5.435273;
 tar_x[111]=22.508766;
 tar_y[111]=-5.277183;
 tar_x[112]=22.633007;
 tar_y[112]=-5.118927;
 tar_x[113]=22.770393;
 tar_y[113]=-4.906385;
 tar_x[114]=22.881706;
 tar_y[114]=-4.689804;
 tar_x[115]=22.964566;
 tar_y[115]=-4.503335;
 tar_x[116]=23.018765;
 tar_y[116]=-4.310498;
 tar_x[117]=23.075345;
 tar_y[117]=-4.110533;
 tar_x[118]=23.102517;
 tar_y[118]=-3.910508;
 tar_x[119]=23.130067;
 tar_y[119]=-3.691475;
 tar_x[120]=23.134321;
 tar_y[120]=-3.441666;
 tar_x[121]=23.110453;
 tar_y[121]=-3.222755;
 tar_x[122]=23.076021;
 tar_y[122]=-3.015805;
 tar_x[123]=23.027457;
 tar_y[123]=-2.801291;
 tar_x[124]=22.961177;
 tar_y[124]=-2.612146;
 tar_x[125]=22.884072;
 tar_y[125]=-2.422410;
 tar_x[126]=22.784833;
 tar_y[126]=-2.239316;
 tar_x[127]=22.665252;
 tar_y[127]=-2.059391;
 tar_x[128]=22.536312;
 tar_y[128]=-1.885297;
 tar_x[129]=22.382275;
 tar_y[129]=-1.724099;
 tar_x[130]=22.224246;
 tar_y[130]=-1.570874;
 tar_x[131]=22.049032;
 tar_y[131]=-1.435986;
 tar_x[132]=21.858797;
 tar_y[132]=-1.313897;
 tar_x[133]=21.655594;
 tar_y[133]=-1.206308;
 tar_x[134]=21.443669;
 tar_y[134]=-1.116617;
 tar_x[135]=21.243311;
 tar_y[135]=-1.036237;
 tar_x[136]=21.026489;
 tar_y[136]=-0.965919;
 tar_x[137]=20.806638;
 tar_y[137]=-0.913141;
 tar_x[138]=20.544737;
 tar_y[138]=-0.850751;
 tar_x[139]=20.318038;
 tar_y[139]=-0.821413;
 tar_x[140]=20.051565;
 tar_y[140]=-0.792580;
 tar_x[141]=19.667202;
 tar_y[141]=-0.767119;
 tar_x[142]=19.464579;
 tar_y[142]=-0.758199;
 tar_x[143]=19.260879;
 tar_y[143]=-0.752988;
 tar_x[144]=19.054562;
 tar_y[144]=-0.741729;
 tar_x[145]=18.846994;
 tar_y[145]=-0.737948;
 tar_x[146]=18.643747;
 tar_y[146]=-0.726163;
 tar_x[147]=18.443367;
 tar_y[147]=-0.719652;
 tar_x[148]=18.242817;
 tar_y[148]=-0.711029;
 tar_x[149]=18.032761;
 tar_y[149]=-0.701350;
 tar_x[150]=17.828273;
 tar_y[150]=-0.694156;
 tar_x[151]=17.622927;
 tar_y[151]=-0.688206;
 tar_x[152]=17.408104;
 tar_y[152]=-0.676410;
 tar_x[153]=17.208149;
 tar_y[153]=-0.669529;
 tar_x[154]=17.002546;
 tar_y[154]=-0.665308;
 tar_x[155]=16.798307;
 tar_y[155]=-0.654379;
 tar_x[156]=16.591701;
 tar_y[156]=-0.649839;
 tar_x[157]=16.377688;
 tar_y[157]=-0.638821;
 tar_x[158]=16.166943;
 tar_y[158]=-0.626485;
 tar_x[159]=15.949740;
 tar_y[159]=-0.614972;
 tar_x[160]=15.732717;
 tar_y[160]=-0.610495;
 tar_x[161]=15.524184;
 tar_y[161]=-0.600450;
 tar_x[162]=15.315088;
 tar_y[162]=-0.597695;
 tar_x[163]=15.102939;
 tar_y[163]=-0.586393;
 tar_x[164]=14.896489;
 tar_y[164]=-0.579757;
 tar_x[165]=14.689480;
 tar_y[165]=-0.570261;
 tar_x[166]=14.483502;
 tar_y[166]=-0.567042;
 tar_x[167]=14.270817;
 tar_y[167]=-0.564467;
 tar_x[168]=14.064652;
 tar_y[168]=-0.559703;
 tar_x[169]=13.850120;
 tar_y[169]=-0.543264;
 tar_x[170]=13.635650;
 tar_y[170]=-0.537504;
 tar_x[171]=13.420607;
 tar_y[171]=-0.539304;
 tar_x[172]=13.218689;
 tar_y[172]=-0.539035;
 tar_x[173]=13.001247;
 tar_y[173]=-0.535159;
 tar_x[174]=12.797490;
 tar_y[174]=-0.528012;
 tar_x[175]=12.581933;
 tar_y[175]=-0.515549;
 tar_x[176]=12.376059;
 tar_y[176]=-0.505148;
 tar_x[177]=12.160700;
 tar_y[177]=-0.502132;
 tar_x[178]=11.959061;
 tar_y[178]=-0.494258;
 tar_x[179]=11.757901;
 tar_y[179]=-0.487346;
 tar_x[180]=11.551954;
 tar_y[180]=-0.485873;
 tar_x[181]=11.346325;
 tar_y[181]=-0.475089;
 tar_x[182]=11.133330;
 tar_y[182]=-0.465795;
 tar_x[183]=10.927163;
 tar_y[183]=-0.458297;
 tar_x[184]=10.717195;
 tar_y[184]=-0.456008;
 tar_x[185]=10.509885;
 tar_y[185]=-0.453099;
 tar_x[186]=10.310047;
 tar_y[186]=-0.440650;
 tar_x[187]=10.096440;
 tar_y[187]=-0.439305;
 tar_x[188]=9.892590;
 tar_y[188]=-0.440354;
 tar_x[189]=9.683606;
 tar_y[189]=-0.435224;
 tar_x[190]=9.473487;
 tar_y[190]=-0.426810;
 tar_x[191]=9.272576;
 tar_y[191]=-0.433409;
 tar_x[192]=9.065353;
 tar_y[192]=-0.433118;
 tar_x[193]=8.862476;
 tar_y[193]=-0.428204;
 tar_x[194]=8.658026;
 tar_y[194]=-0.427232;
 tar_x[195]=8.456867;
 tar_y[195]=-0.425912;
 tar_x[196]=8.252252;
 tar_y[196]=-0.416410;
 tar_x[197]=8.047331;
 tar_y[197]=-0.412007;
 tar_x[198]=7.845874;
 tar_y[198]=-0.415145;
 tar_x[199]=7.635596;
 tar_y[199]=-0.407460;
 tar_x[200]=7.427712;
 tar_y[200]=-0.413151;
 tar_x[201]=7.227500;
 tar_y[201]=-0.410500;
 tar_x[202]=7.024535;
 tar_y[202]=-0.406462;
 tar_x[203]=6.817209;
 tar_y[203]=-0.400878;
 tar_x[204]=6.607246;
 tar_y[204]=-0.399200;
 tar_x[205]=6.401973;
 tar_y[205]=-0.399251;
 tar_x[206]=6.192973;
 tar_y[206]=-0.393707;
 tar_x[207]=5.989141;
 tar_y[207]=-0.394176;
 tar_x[208]=5.782005;
 tar_y[208]=-0.382318;
 tar_x[209]=5.577341;
 tar_y[209]=-0.380272;
 tar_x[210]=5.373177;
 tar_y[210]=-0.379015;
 tar_x[211]=5.169882;
 tar_y[211]=-0.372713;
 tar_x[212]=4.962129;
 tar_y[212]=-0.374029;
 tar_x[213]=4.757635;
 tar_y[213]=-0.370459;
 tar_x[214]=4.550556;
 tar_y[214]=-0.367820;
 tar_x[215]=4.343282;
 tar_y[215]=-0.361750;
 tar_x[216]=4.135874;
 tar_y[216]=-0.361467;
 tar_x[217]=3.931083;
 tar_y[217]=-0.358885;
 tar_x[218]=3.723568;
 tar_y[218]=-0.355260;
 tar_x[219]=3.519907;
 tar_y[219]=-0.358531;
 tar_x[220]=3.320081;
 tar_y[220]=-0.349685;
 tar_x[221]=3.114342;
 tar_y[221]=-0.348346;
 tar_x[222]=2.911752;
 tar_y[222]=-0.344448;
 tar_x[223]=2.706287;
 tar_y[223]=-0.343868;
 tar_x[224]=2.503886;
 tar_y[224]=-0.331041;
 tar_x[225]=2.299535;
 tar_y[225]=-0.336284;
 tar_x[226]=2.091092;
 tar_y[226]=-0.341298;
 tar_x[227]=1.887067;
 tar_y[227]=-0.325557;
 tar_x[228]=1.681838;
 tar_y[228]=-0.326173;
 tar_x[229]=1.476524;
 tar_y[229]=-0.327121;
 tar_x[230]=1.275344;
 tar_y[230]=-0.319825;
 tar_x[231]=1.067429;
 tar_y[231]=-0.317423;
 tar_x[232]=0.867088;
 tar_y[232]=-0.313221;
 tar_x[233]=0.664870;
 tar_y[233]=-0.317316;
 tar_x[234]=0.463066;
 tar_y[234]=-0.315229;
 tar_x[235]=0.259262;
 tar_y[235]=-0.322535;
 tar_x[236]=0.056430;
 tar_y[236]=-0.322391;
 tar_x[237]=-0.152402;
 tar_y[237]=-0.327018;
 tar_x[238]=-0.362030;
 tar_y[238]=-0.326713;
 tar_x[239]=-0.563523;
 tar_y[239]=-0.330971;
 tar_x[240]=-0.770542;
 tar_y[240]=-0.332714;

for(count_path=0;count_path<241;count_path++)
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
  num_=min_ind;
  target_x=tar_x[num_];
  target_y=tar_y[num_];
  count_init++;
  }

  //计算AGV距离轨迹的最新间距，更新预瞄距离
  d_min_refersh(x,y);
  if(d_min<0.5)
  {
    d_ym=2;
  }
  else
  {
    d_ym=d_min+2;
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
     if(num_>240)
     {
        num_=240;
        break;
     }
      target_x=tar_x[num_];
      target_y=tar_y[num_];
     d=sqrt((target_x-x)*(target_x-x)+(target_y-y)*(target_y-y));
  }
  
//计算角速度
if(theta<40 && theta>20)
 {
    msg_twist.angular.z=2*3*theta/400;
 }
 if(theta<20&&theta>0)
 {
   msg_twist.angular.z=2*3*theta/400;
 }
 if(theta==40 || theta>40)
 {
   msg_twist.angular.z=2*0.3;
 }
 if(theta<0 && theta>-20)
 {
   msg_twist.angular.z=2*3*theta/400;
 }
 if(theta<-20 && theta>-40)
 {
    msg_twist.angular.z=2*3*theta/400;
 }
 if(theta==20)
 {
   msg_twist.angular.z=2*0.15;
 }
 if(theta==-20)
 {
   msg_twist.angular.z=-0.3;
 }
 if(theta<-40 || theta==-40)
 {
    msg_twist.angular.z=-0.6;
 }

 //设定线速度
  msg_twist.linear.x=2*0.4;

  //轨迹终点停车
      if(num_==240 && d<0.3)
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

    /*marker3.scale.x = d_ym;
    marker3.scale.y = d_ym;
    marker3.scale.z = 0.4;*/

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
printf("delta_l:%f\n,delta_r:%f\n",angle_delta_l,angle_delta_r);
 //ROS_INFO("d:%f\n theta1:%f\n theta2:%f\n num:%d\n",d,theta1,theta2,num_);
 //ROS_INFO("theta1:%f,theta3:%f,v1:%f,v3:%f",theta1,theta3,v1,v3);
 //printf(" theta:%f\n delta_angle_l:%f\n delta_angle_r:%f\n d:%f\n num:%d\n,d_ym:%f",theta,angle_delta_l,angle_delta_r,d,num_,d_ym);
 pub_.publish(msg_twist);
 pub_path.publish(msg_path);
 pub_path_target.publish(msg_target);
 marker_pub.publish(array);
}

void target_init(const double& x_init,const double& y_init)
{
  d_min=sqrt((tar_x[0]-x_init)*(tar_x[0]-x_init)+(tar_y[0]-y_init)*(tar_y[0]-y_init));
  min_ind=0;
for(count_d=1;count_d<241;count_d++)
  {
    d_temp=sqrt((tar_x[count_d]-x_init)*(tar_x[count_d]-x_init)+(tar_y[count_d]-y_init)*(tar_y[count_d]-y_init));
    if(d_temp<d_min)
    {
      d_min=d_temp;
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
for(count_d=1;count_d<241;count_d++)
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
  for(count_d=1;count_d<120;count_d++)
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
       d_min=sqrt((tar_x[120]-x_init)*(tar_x[120]-x_init)+(tar_y[120]-y_init)*(tar_y[120]-y_init));
       min_ind=120;
  for(count_d=121;count_d<241;count_d++)
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