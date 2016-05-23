/*
  CwruStim.h - Library for muscle stim board for HNPv2 Project.
  Created by Lu Li (lxl361@case), Aug, 2015.
  Version 1.1
  Online Doc: https://goo.gl/s20iH4
  Repo: https://github.com/lilulab/CwruStim_beta
*/

#ifndef CwruStim_h
#define CwruStim_h

#include <Arduino.h>

#include <CwruStimConst.h>
#include "StimPattern/Gait_VCK5.h" // VCK5's walking pattern

/*
  HNPv2 NTREK Embedded Controller Board(ECB) 
  Support two channels for muscle stimulation board UART communication.
  UART1 (TX1 and RX1) for stim board 1
  UART3 (TX3 and RX3) for stim board 3
 */ 

class Stim
{
  public:

    Stim(int uart_channel_id); // Stim constructor and UART selector

    // Stim board setup function
    int init(int mode); // Initialize Stim Board "\']/;Ub=/\\x00/g;Vb=/\'/g;Wb=/"/g;Xb=/>/g;Yb=/")&&(a=a.replace(Xb,">"));-1!=a.indexOf(\'"\')&&(a=a.replace(Wb,"""));-1!=a.indexOf("\'")&&(a=a.replace(Vb,"'"));-1!=a.indexOf("\\x00")&&(a=a.replace(Ub," "));return a};_.ac=function(a){var c=typeof a;return"object"==c&&null!=a||"function"==c};\nbc=function(a){if(a&&"number"==typeof a.length){if(_.ac(a))return"function"==typeof a.item||"string"==typeof a.item;if(_.fa(a))return"function"==typeof a.item}return!1};cc={cellpadding:"cellPadding",cellspacing:"cellSpacing",colspan:"colSpan",frameborder:"frameBorder",height:"height",maxlength:"maxLength",role:"role",rowspan:"rowSpan",type:"type",usemap:"useMap",valign:"vAlign",width:"width"};dc="constructor hasOwnProperty isPrototypeOf propertyIsEnumerable toLocaleString toString valueOf".split(" ");\n_.ec=function(a,c,d){for(var e in a)c.call(d,a[e],e,a)};_.fc=function(a){var c=a.length;if(0");d=d.join("")}d=a.createElement(d);e&&(_.u(e)?d.className=e:_.ea(e)?d.className=e.join(" "):_.jc(d,e));2=a.keyCode)a.keyCode=-1}catch(c){}};_.Bc.prototype.U=function(){return this.b};\nvar Ec;_.Cc="closure_listenable_"+(1E6*Math.random()|0);_.Dc=function(a){return!(!a||!a[_.Cc])};Ec=0;var Fc=function(a,c,d,e,f){this.listener=a;this.b=null;this.src=c;this.type=d;this.gc=!!e;this.Ac=f;this.key=++Ec;this.Fb=this.fc=!1},Gc=function(a){a.Fb=!0;a.listener=null;a.b=null;a.src=null;a.Ac=null};var Hc=function(a){this.src=a;this.b={};this.o=0},Jc,Ic;Hc.prototype.add=function(a,c,d,e,f){var g=a.toString();a=this.b[g];a||(a=this.b[g]=[],this.o++);var h=Ic(a,c,e,f);-1d.keyCode||void 0!=d.returnValue)){a:{var g=!1;if(0==d.keyCode)try{d.keyCode=-1;break a}catch(h){g=!0}if(g||void 0==d.returnValue)d.returnValue=!0}d=[];for(g=e.o;g;g=g.parentNode)d.push(g);for(var g=a.type,l=d.length-1;!e.C&&0>>0);_.Qc=function(a){if(_.fa(a))return a;a[ad]||(a[ad]=function(c){return a.handleEvent(c)});return a[ad]};\nvar ed;_.bd=function(a){_.z.call(this);this.Y=a;this.fa={}};_.y(_.bd,_.z);var cd=[];_.bd.prototype.b=function(a,c,d,e){return dd(this,a,c,d,e)};_.bd.prototype.J=function(a,c,d,e,f){return dd(this,a,c,d,e,f)};var dd=function(a,c,d,e,f,g){_.ea(d)||(d&&(cd[0]=d.toString()),d=cd);for(var h=0;h2*this.w&&jd(this),!0):!1};var jd=function(a){if(a.w!=a.b.length){for(var c=0,d=0;c=e.b.length)throw _.gd;var f=e.b[c++];return a?f:e.o[f]};return f};var kd=function(a,c){return Object.prototype.hasOwnProperty.call(a,c)};\n_.ld=function(a){if("function"==typeof a.La)return a.La();if(_.u(a))return a.split("");if(_.hc(a)){for(var c=[],d=a.length,e=0;ec)throw Error("D`"+c);a.A=c}else a.A=null;return a};_.$d.prototype.Xa=function(){return this.C};_.de=function(a,c,d){a.C=d?fe(c,!0):c;return a};_.ee=function(a,c,d){c instanceof ge?(a.Fa=c,me(a.Fa,a.b)):(d||(c=he(c,ne)),a.Fa=new ge(c,0,a.b));return a};\nfe=function(a,c){return a?c?(0,window.decodeURI)(a.replace(/%25/g,"%2525")):(0,window.decodeURIComponent)(a):""};he=function(a,c,d){return _.u(a)?(a=(0,window.encodeURI)(a).replace(c,oe),d&&(a=a.replace(/%25([0-9a-fA-F]{2})/g,"%$1")),a):null};oe=function(a){a=a.charCodeAt(0);return"%"+(a>>4&15).toString(16)+(a&15).toString(16)};ie=/[#\\/\\?@]/g;ke=/[\\#\\?:]/g;je=/[\\#\\?]/g;ne=/[\\#\\?@]/g;le=/#/g;ge=function(a,c,d){this.w=this.b=null;this.o=a||null;this.A=!!d};\npe=function(a){a.b||(a.b=new _.id,a.w=0,a.o&&Zd(a.o,function(c,d){a.add((0,window.decodeURIComponent)(c.replace(/\\+/g," ")),d)}))};ge.prototype.add=function(a,c){pe(this);this.o=null;a=qe(this,a);var d=this.b.get(a);d||this.b.set(a,d=[]);d.push(c);this.w++;return this};ge.prototype.remove=function(a){pe(this);a=qe(this,a);return kd(this.b.o,a)?(this.o=null,this.w-=this.b.get(a).length,this.b.remove(a)):!1};ge.prototype.clear=function(){this.b=this.o=null;this.w=0};\nge.prototype.Xb=function(){pe(this);return 0==this.w};var re=function(a,c){pe(a);c=qe(a,c);return kd(a.b.o,c)};_.k=ge.prototype;_.k.Ka=function(){pe(this);for(var a=this.b.La(),c=this.b.Ka(),d=[],e=0;e=arguments.length?_.pa.slice.call(a,c):_.pa.slice.call(a,c,d)};\n_.cg=function(a){return _.ac(a)&&1==a.nodeType};dg=function(){return(_.K("Chrome")||_.K("CriOS"))&&!_.nb()&&!_.K("Edge")};eg=function(a,c,d,e){_.pa.splice.apply(a,_.bg(arguments,1))};fg=0;_.gg=function(a){try{return a&&a.activeElement}catch(c){}return null};_.hg=function(a,c){return a==c?!0:a&&c?a.width==c.width&&a.height==c.height:!1};_.ig=function(a,c,d){a.b||(a.b={});if(!a.b[d]){for(var e=_.E(a,d),f=[],g=0;g=a||96=a||65=a||(_.M||_.tb)&&0==a)return!0;switch(a){case 32:case 63:case 64:case 107:case 109:case 110:case 111:case 186:case 59:case 189:case 187:case 61:case 188:case 190:case 191:case 192:case 222:case 219:case 220:case 221:return!0;default:return!1}};_.ug=function(a){if(_.ub)a=wg(a);else if(_.yb&&_.M)a:switch(a){case 93:a=91;break a}return a}; wg=function(a){switch(a){case 61:return 187;case 59:return 186;case 173:return 189;case 224:return 91;case 0:return 224;default:return a}};\n_.xg=function(a,c,d,e){this.top=a;this.right=c;this.bottom=d;this.left=e};_.k=_.xg.prototype;_.k.getHeight=function(){return this.bottom-this.top};_.k.clone=function(){return new _.xg(this.top,this.right,this.bottom,this.left)};_.k.contains=function(a){return this&&a?a instanceof _.xg?a.left>=this.left&&a.right=this.top&&a.bottom=this.left&&a.x=this.top&&a.y=a.left+a.width&&this.top=a.top+a.height:a.x>=this.left&&a.x=this.top&&a.yc||c>_.bh(this))throw Error("O");this.G&&this.B||(this.G={},this.B=[]);if(a.w==this){var e=a.getId();this.G[e]=a;_.rc(this.B,a)}else{var e=this.G,f=a.getId();if(f in e)throw Error("c`"+f);e[f]=a}Yg(a,this);eg(this.B,c,0,a);a.ja&&this.ja&&a.w==this?(d=this.o,c=d.childNodes[c]||null,c!=a.P()&&d.insertBefore(a.P(),c)):d?(this.o||this.mb(),c=_.ch(this,c+1),Zg(a,this.o,c?c.o:null)):this.ja&&!a.ja&&a.o&&a.o.parentNode&&1==a.o.parentNode.nodeType&&\na.ka()};_.bh=function(a){return a.B?a.B.length:0};_.ch=function(a,c){return a.B?a.B[c]||null:null};_.ah=function(a,c,d){a.B&&(0,_.ra)(a.B,c,d)};_.U.prototype.removeChild=function(a,c){if(a){var d=_.u(a)?a:a.getId();a=this.G&&d?jg(this.G,d)||null:null;if(d&&a){var e=this.G;d in e&&delete e[d];_.rc(this.B,a);c&&(a.ua(),a.o&&_.Sf(a.o));Yg(a,null)}}if(!a)throw Error("P");return a};\n_.dh=function(a,c,d){_.z.call(this);this.o=a;this.A=c||0;this.w=d;this.b=(0,_.v)(this.B,this)};_.y(_.dh,_.z);_.dh.prototype.ea=0;_.dh.prototype.O=function(){_.dh.H.O.call(this);_.eh(this);delete this.o;delete this.w};_.dh.prototype.start=function(a){_.eh(this);this.ea=_.Vd(this.b,_.n(a)?a:this.A)};_.eh=function(a){0!=a.ea&&_.m.clearTimeout(a.ea);a.ea=0};_.dh.prototype.B=function(){this.ea=0;this.o&&this.o.call(this.w)};\nvar hh,jh;_.fh={};hh=null;_.ih=function(a){a=_.lg(a);delete _.fh[a];_.kg(_.fh)&&hh&&_.eh(hh)};_.kh=function(){hh||(hh=new _.dh(function(){jh()},20));var a=hh;0!=a.ea||a.start()};jh=function(){var a=(0,_.w)();_.ec(_.fh,function(c){_.lh(c,a)});_.kg(_.fh)||_.kh()};\n_.lh=function(a,c){a.w=(c-a.B)/(a.M-a.B);1a};Xj=function(a){a=a.getAttributeNode("tabindex");return null!=a&&a.specified};Yj=function(a,c,d,e){if(null!=a)for(a=a.firstChild;a;){if(c(a)&&(d.push(a),e)||Yj(a,c,d,e))return!0;a=a.nextSibling}return!1};\n_.Zj=function(a,c){for(;a&&1!=a.nodeType;)a=c?a.nextSibling:a.previousSibling;return a};ak=function(a,c){var d=[];return Yj(a,c,d,!0)?d[0]:void 0};_.bk=function(a){var c;if(_.Nb&&!(_.L&&_.N("9")&&!_.N("10")&&_.m.SVGElement&&a instanceof _.m.SVGElement)&&(c=a.parentElement))return c;c=a.parentNode;return _.cg(c)?c:null};ck=function(a){return _.n(a.previousElementSibling)?a.previousElementSibling:_.Zj(a.previousSibling,!1)}; dk=function(a){return _.n(a.nextElementSibling)?a.nextElementSibling:_.Zj(a.nextSibling,!0)};_.ek=function(a){return _.u(a)?window.document.getElementById(a):a};\nvar fk,hk;fk=function(){};_.gk=new fk;hk=["click",_.ub?"keypress":"keydown","keyup"];fk.prototype.b=function(a,c,d,e,f){var g=function(a){var d=_.Qc(c),f=_.cg(a.target)?a.target.getAttribute("role")||null:null;"click"==a.type&&_.Vj(a)?d.call(e,a):13!=a.keyCode&&3!=a.keyCode||"keyup"==a.type?32!=a.keyCode||"keyup"!=a.type||"button"!=f&&"tab"!=f||(d.call(e,a),a.preventDefault()):(a.type="keypress",d.call(e,a))};g.o=c;g.b=e;f?f.b(a,hk,g,d):_.P(a,hk,g,d)};\n_.ik=_.L||_.ub&&_.N("1.9.3");_.jk=function(){_.Q.call(this);this.w=[];this.oa=[];this.L=[];this.T=this.X=this.W=!1};_.y(_.jk,_.Q);_.k=_.jk.prototype;_.k.Tc=function(a){this.oa.push(a);this.W=a.X=!0};_.k.init=function(){if(!this.T){for(var a,c=0;a=this.w[c];c++)this.Dc(a);this.T=!0}};_.k.Dc=function(a){this.W&&(_.P(a.o,"mousedown",a.Ce,!1,a),this.$&&_.S(a.o,this.$));this.X&&this.aa&&_.S(a.o,this.aa)};_.k.jc=function(a){this.W&&(_.Xc(a.o,"mousedown",a.Ce,!1,a),this.$&&_.T(a.o,this.$));this.X&&this.aa&&_.T(a.o,this.aa);a.da()};\n_.k.je=function(a){var c=a.w?null:this.F;if(c&&c.w){var d=a.clientX;a=a.clientY;var e=_.Yf(_.Zf(this.M)),f=d+e.x,e=a+e.y,g;this.D&&(g=this.D(c.o,c.b,f,e));this.N(new _.kk("drag",0,this.o,0,c.o,0,d,a));c.w.N(new _.kk("drop",0,this.o,0,c.o,0,d,a,0,0,g))}this.N(new _.kk("dragend",0,this.o));_.Xc(this.A,"drag",this.Ee,!1,this);_.Xc(this.A,"end",this.je,!1,this);_.Xc(_.Vf(this.o.w).body,"selectstart",this.Qe);for(c=0;d=this.L[c];c++)_.Xc(d.b,"scroll",this.Wd,!1,this),d.o=[];this.A.da();_.Sf(this.M);delete this.o;\ndelete this.M;delete this.A;delete this.R;delete this.F};\n_.k.Ee=function(a){var c,d=_.Yf(_.Zf(this.M));c=new _.Mf(a.clientX+d.x,a.clientY+d.y);var d=c.x,e=c.y,f=this.F,g;if(f){this.D&&f.w&&(g=this.D(f.o,f.b,d,e));if(f.b.contains(c)&&g==this.ba)return;f.w&&(this.N(new _.kk("dragout",0,this.o,0,f.o)),f.w.N(new _.kk("dragout",0,this.o,0,f.o,0,void 0,void 0,0,0,this.ba)));this.ba=g;this.F=null}if(this.B.contains(c)){a:{for(var h=0;f=this.R[h];h++)if(f.b.contains(c))if(f.A){if(f.A.w.contains(c)){c=f;break a}}else{c=f;break a}c=null}if((f=this.F=c)&&f.w)this.D&&\n(g=this.D(f.o,f.b,d,e)),d=new _.kk("dragover",0,this.o,0,f.o),d.w=g,this.N(d),f.w.N(new _.kk("dragover",0,this.o,0,f.o,0,a.clientX,a.clientY,0,0,g));else if(!f){this.b||(this.b=new _.lk(this.B.clone()));a=this.b.b;a.top=this.B.top;a.right=this.B.right;a.bottom=this.B.bottom;a.left=this.B.left;for(g=0;f=this.R[g];g++)c=f.b,f.A&&(f=f.A.w,c=new _.xg(Math.max(c.top,f.top),Math.min(c.right,f.right),Math.min(c.bottom,f.bottom),Math.max(c.left,f.left))),f=null,d>=c.right?f=c.right>a.left?c.right:a.left:\nd=c.bottom?h=c.bottom>a.top?c.bottom:a.top:eMath.abs(h-e)?h=null:f=null),null!==f?f=c.G&&c.cancel())}this.K?this.K.call(this.J,this):this.F=!0;this.b||this.M()}};_.Mk.prototype.U=function(a,c){this.D=!1;Nk(this,a,c)};\nvar Nk=function(a,c,d){a.b=!0;a.w=d;a.A=!c;Ok(a)},Rk=function(a){if(a.b){if(!a.F)throw new Pk;a.F=!1}};_.Mk.prototype.hb=function(a){Rk(this);Nk(this,!0,a)};_.Mk.prototype.M=function(){var a=new Sk;Rk(this);Nk(this,!1,a)};_.Mk.prototype.addCallback=function(a,c){return Tk(this,a,null,c)};var Tk=function(a,c,d,e){a.B.push([c,d,e]);a.b&&Ok(a);return a};\n_.Mk.prototype.then=function(a,c,d){var e,f,g=new _.Ed(function(a,c){e=a;f=c});Tk(this,e,function(a){a instanceof Sk?g.cancel():f(a)});return g.then(a,c,d)};_.Bd(_.Mk);\nvar Uk=function(a){return(0,_.va)(a.B,function(a){return _.fa(a[1])})},Ok=function(a){if(a.C&&a.b&&Uk(a)){var c=a.C,d=Vk[c];d&&(_.m.clearTimeout(d.ea),delete Vk[c]);a.C=0}a.o&&(a.o.G--,delete a.o);for(var c=a.w,e=d=!1;a.B.length&&!a.D;){var f=a.B.shift(),g=f[0],h=f[1],f=f[2];if(g=a.A?h:g)try{var l=g.call(f||a.J,c);_.n(l)&&(a.A=a.A&&(l==c||l instanceof Error),a.w=c=l);if(_.Cd(c)||"function"===typeof _.m.Promise&&c instanceof _.m.Promise)e=!0,a.D=!0}catch(p){c=p,a.A=!0,Uk(a)||(d=!0)}}a.w=c;e&&(l=(0,_.v)(a.U,\na,!0),e=(0,_.v)(a.U,a,!1),c instanceof _.Mk?(Tk(c,l,e),c.V=!0):c.then(l,e));d&&(c=new Wk(c),Vk[c.ea]=c,a.C=c.ea)},Pk=function(){_.la.call(this)};_.y(Pk,_.la);Pk.prototype.message="Deferred has already fired";Pk.prototype.name="AlreadyCalledError";var Sk=function(){_.la.call(this)};_.y(Sk,_.la);Sk.prototype.message="Deferred was canceled";Sk.prototype.name="CanceledError";var Wk=function(a){this.ea=_.m.setTimeout((0,_.v)(this.o,this),0);this.b=a}; Wk.prototype.o=function(){delete Vk[this.ea];throw this.b;};var Vk={};\n\n}catch(e){_._DumpException(e)}\ntry{\nvar fl,il,jl,ol,pl,ql;_.Xk=function(a){_.C(this,a,0,-1,null)};_.y(_.Xk,_.B);_.Yk=function(){return _.F(_.J(),_.Xk,11)};_.Zk=function(a){return null!=_.E(a,2)?_.E(a,2):.001};_.$k=function(a){_.C(this,a,0,-1,null)};_.y(_.$k,_.B);var al=function(a){return null!=_.E(a,3)?_.E(a,3):1},bl=function(a){return null!=_.E(a,2)?_.E(a,2):1E-4},cl=function(a){_.C(this,a,0,-1,null)};_.y(cl,_.B);_.el=function(){var a=_.dl();return _.E(a,9)};fl=function(a){_.C(this,a,0,-1,null)};_.y(fl,_.B);\n_.gl=function(a){return _.E(a,10)};_.hl=function(a){return _.E(a,5)};il=0;jl=[];_.kl=function(){this.data={}};_.kl.prototype.b=function(){window.console&&window.console.log&&window.console.log("Log data: ",this.data)};_.kl.prototype.o=function(a){var c=[],d;for(d in this.data)c.push((0,window.encodeURIComponent)(d)+"="+(0,window.encodeURIComponent)(String(this.data[d])));return("atyp=i&zx="+(new Date).getTime()+"&"+c.join("&")).substr(0,a)};\nvar ll=function(a){var c=new window.Image,d=il;c.onerror=c.onload=c.onabort=function(){d in jl&&delete jl[d]};jl[il++]=c;c.src=a},ml=function(a,c){this.data={};var d=_.F(a,_.Ma,8)||new _.Ma;this.data.ei=_.H(_.gl(a));this.data.ogf=_.H(_.E(d,3));var e;e=window.google&&window.google.sn?/.*hp$/.test(window.google.sn)?!1:!0:_.G(_.E(a,7));this.data.ogrp=e?"1":"";this.data.ogv=_.H(_.E(d,6))+"."+_.H(_.E(d,7));this.data.ogd=_.H(_.E(a,21));this.data.ogc=_.H(_.E(a,20));this.data.ogl=_.H(_.hl(a));c&&(this.data.oggv=\nc)};_.y(ml,_.kl);_.nl=function(a){var c="//www.google.com/gen_204?",c=c+a.o(2040-c.length);ll(c)};ol=null;pl=function(a){return(a+"").replace(".","%2E").replace(",","%2C")};ql=[1,2,3,4,5,6,9,10,11,13,14,28,29,30,34,35,37,38,39,40,41,42,43,48,49,50,51,52,53,500];\n_.rl=function(a,c,d,e,f){ml.call(this,a,c);_.kc(this.data,{jexpid:_.H(_.E(a,9)),srcpg:"prop="+_.H(_.E(a,6)),jsr:Math.round(1/e),emsg:d.name+":"+d.message});if(f){f._sn&&(f._sn="og."+f._sn);for(var g in f)this.data[(0,window.encodeURIComponent)(g)]=f[g]}};_.y(_.rl,ml);var sl=function(a){if(!ol){ol={};for(var c=0;cc&&(c=0);a.b.style.width=c+"px";d=a.b.offsetWidth-d;a.b.style.width=d+"px";return d-e},dm=function(a){var c=a.b.style.width;a.b.style.width="";return c};\nvar hm=function(a,c,d){var e;void 0==e&&(e=-1);return{className:a,jb:{yc:c||0,Mc:d||0,Pb:e}}},im={className:"gb_Hc",items:[hm("gb_0a"),hm("gb_Vc"),hm("gb_Jb",0,2),hm("gb_Wc"),hm("gb_ga",1,1)],ib:[{className:"gb_ga",items:[hm("gb_md",0,1),hm("gb_Fb",0,1)],ib:[function(a){a=a.gb_md;var c;if(a)c=a.P();else{c=window.document.querySelector(".gb_md");if(!c)return null;a=new am(c)}c=c.querySelectorAll(".gb_L");for(var d=0;dc&&(a.A=0,a.B(new Rl))}};_.mm.prototype.fa=function(a){this.D.push(a)};_.mm.prototype.K=function(a){var c=nm().es.h;this.Ud=c+a;for(a=0;a(0,window.parseFloat)(a[1])&&(this.o=!0)};_.y(vm,_.z);\nvar wm=function(a,c,d){if(!a.o)if(d instanceof Array)for(var e in d)wm(a,c,d[e]);else{e=(0,_.v)(a.G,a,c);var f=a.D+d;a.D++;c.setAttribute("data-eqid",f);a.B[f]=e;c&&c.addEventListener?c.addEventListener(d,e,!1):c&&c.attachEvent?c.attachEvent("on"+d,e):a.C.log(Error("aa`"+c))}};\nvm.prototype.A=function(a,c){if(this.o)return null;if(c instanceof Array){var d=null,e;for(e in c){var f=this.A(a,c[e]);f&&(d=f)}return d}d=null;this.b&&this.b.type==c&&this.w==a&&(d=this.b,this.b=null);if(e=a.getAttribute("data-eqid"))a.removeAttribute("data-eqid"),(e=this.B[e])?a.removeEventListener?a.removeEventListener(c,e,!1):a.detachEvent&&a.detachEvent("on"+c,e):this.C.log(Error("ba`"+a));return d};\nvm.prototype.G=function(a,c){this.b=c;this.w=a;c.preventDefault?c.preventDefault():c.returnValue=!1};var xm=function(a){_.z.call(this);this.C=a;this.A=this.b=null;this.o={};this.B={};this.w={}};_.y(xm,_.z);_.k=xm.prototype;_.k.Oe=function(a){a&&this.b&&a!=this.b&&this.b.close();this.b=a};_.k.Be=function(a){a=this.w[a]||a;return this.b==a};_.k.uh=function(a){this.A=a};_.k.Ae=function(a){return this.A==a};_.k.Yc=function(){this.b&&this.b.close();this.b=null};\n_.k.gf=function(a){this.b&&this.b.getId()==a&&this.Yc()};_.k.Mb=function(a,c,d){this.o[a]=this.o[a]||{};this.o[a][c]=this.o[a][c]||[];this.o[a][c].push(d)};_.k.Wc=function(a,c){var d=c.getId();if(this.o[a]&&this.o[a][d])for(var e=0;eMy Account UART
    int config(int setting); // Configure Stim Board via UART
    int update(int type, int pattern, uint16_t cycle_percentage); // Update Stim pattern via UART

    uint16_t _Gait_LUT_VCK5[NUM_GAIT_VCK5_BOARD][NUM_PATTERN][NUM_PATTERN_PARAM];//[board][pattern][PP/PW/AMP/IPI]
    int gait_LUT_builder(void); 

    int start(uint8_t sync_signal); // Start Stim board using sync signal command
    int start_multi_schedule(void); // Start multiple scheduler
    // UART lower level stuff
    int serial_write_array(uint8_t buf[],int length);

    int debug_print_states(int id); // Use Serial0 to print debug messages.

    // UECU command sets

    //UECU Halt and reset
    int cmd_halt_rset(uint8_t halt_flag);

    // UECU Delete Schedule
    int cmd_del_sched(uint8_t sched_id);

    // UECU Create Schedule
    int cmd_crt_sched(uint8_t sync_signal, uint16_t duration);

    // UECU Channel Setup 
    int cmd_chan_set( uint8_t port_chn_id, 
                      uint8_t amp_limit,
                      uint8_t pw_limit,
                      uint16_t ip_delay, 
                      uint8_t asp_ratio, 
                      uint8_t anode_cathode);

    // UECU Create Event
    int cmd_crt_evnt( uint8_t sched_id, 
                        uint16_t delay, 
                        uint8_t priority, 
                        uint8_t event_type, 
                        uint8_t port_chn_id,
                        uint8_t pulse_width,
                        uint8_t amplitude,
                        uint8_t zone);

    // UECU Change Event Parameter Command
    int cmd_set_evnt( uint8_t event_id,
                      uint8_t pulse_width,
                      uint8_t amplitude,
                      uint8_t zone);

    // UECU Change Event Schedule Message
    int cmd_chg_evnt_sched(
                      uint8_t event_id,
                      uint8_t sched_id,
                      uint16_t delay,
                      uint8_t priority);

    // UECU Change Schedule Message
    int cmd_set_sched( uint8_t sched_id,
                      uint8_t sync_signal,
                      uint16_t duration); 

    // UECU Sync Message
    int cmd_sync_msg( uint8_t sync_signal); 

    // channel pw gains for dynamic gait control
    // related var: float _current_pw_gains[STIM_CHANNEL_MAX_PERC];
    int set_chan_pw_gain(uint8_t channel, float gain);
    float get_chan_pw_gain(uint8_t channel);

    // channel amp gains for dynamic gait control
    // related var: float _current_amp_gains[STIM_CHANNEL_MAX_PERC];
    int set_chan_amp_gain(uint8_t channel, float gain);
    float get_chan_amp_gain(uint8_t channel);
  
  private:
    // Stim board setup
    int _uart_channel_id;
    int _mode;
    int _setting;
    int _stim_error;

    // Multi scheduler sync
    uint8_t _PERC_8CH_SYNC_MSG[STIM_CHANNEL_MAX_PERC];

    // Pulse width
    uint8_t _current_pulse_width[STIM_CHANNEL_MAX_PERC];

    // Amplitude
    uint8_t _current_amplitude[STIM_CHANNEL_MAX_PERC];

    // Inter phase interval
    uint8_t _current_ipi[STIM_CHANNEL_MAX_PERC];

    // for fixed scheduler
    uint16_t _group_event_count[FIXED_SCHED];//count num of events in one schedule
    uint8_t _schedule_id; //schedule_id 

    // channel pw gains for dynamic gait control
    float _current_pw_gains[STIM_CHANNEL_MAX_PERC];

    // channel amp gains for dynamic gait control
    float _current_amp_gains[STIM_CHANNEL_MAX_PERC];

    // cal check sum byte
    uint8_t checksum(uint8_t vals[], int length);

    // gen pw ranping
    uint8_t get_PW_ramping( int channel_i,
                            const uint16_t (*LUT_PP_t)[12][8],
                            const uint8_t (*LUT_PW_t)[12][8],
                            uint16_t cycle_pp_t);

    // execute command, apply the gain to _current_pulse_width[];
    uint8_t exe_chan_pw_gain(uint8_t channel);

    // execute command, apply the gain to _current_amplitude[];
    uint8_t exe_chan_amp_gain(uint8_t channel);

};

#endif



