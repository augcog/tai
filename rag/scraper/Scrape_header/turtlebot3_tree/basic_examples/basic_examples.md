

TurtleBot3

 {
 "@context" : "http://schema.org",
 "@type" : "Person",
 "name" : "ROBOTIS",
 "url" : "https://emanual.robotis.com",
 "sameAs" : null
 }

$(function() {
 return $("h1, h2, h3, h4, h5, h6").each(function(i, el) {
 var $el, icon, id;
 $el = $(el);
 id = $el.attr('id');
 icon = '<i class="fa fa-link"></i>';
 if (id) {
 return $el.append($("<a />").addClass("header-link").attr("href", "#" + decodeURI(id)).attr("onclick", "copyurl(this);").html(icon));
 }
 });
});

$(function(){
 $('h1').each(function(){
 $(this).addClass('class-h1');
 });
 $('h2').each(function(){
 $(this).addClass('class-h2');
 });
 $('h3').each(function(){
 $(this).addClass('class-h3');
 });
 $('h4').each(function(){
 $(this).addClass('class-h4');
 });
 $('h5').each(function(){
 $(this).addClass('class-h5');
 });
 $('h6').each(function(){
 $(this).addClass('class-h6');
 });
});

$(function() {
 $('h1, h2, h3, h4, h5, h6').each(function(i, e) {
 $(e).nextUntil('h1, h2, h3, h4, h5, h6').wrapAll('<div>');
 });

});

 var toggle\_google = 0;
 var toggle\_search\_tool = 0;
 var toggle\_search\_result = 0;
 var mobile\_width = 960;

 function extendgoogletool() {
 if (toggle\_google == 0) {
 google\_tools.classList.add("extend\_googletools");
 switch\_ko\_en.classList.add("show\_switch\_ko\_en");
 toggle\_google = 1;
 }
 else {
 google\_tools.classList.remove("extend\_googletools");
 switch\_ko\_en.classList.remove("show\_switch\_ko\_en");
 toggle\_google = 0;
 }
 }

 function extendsearchtool() {
 if (toggle\_search\_tool == 0) {
 $('.searchbox').css('display', 'block');
 $('#archive').css('margin-top', '40px');
 toggle\_search\_tool = 1;
 }
 else {
 $('.searchbox').css('display', 'none');
 $('#archive').css('margin-top', '0px');
 toggle\_search\_tool = 0;
 }
 }

 $(document).ready(function() {
 $('.blank').attr('target', '\_blank');
 $('.popup').magnificPopup({
 type: 'ajax',
 alignTop: true,
 overflowY: 'scroll' // as we know that popup content is tall we set scroll overflow by default to avoid jump
 });
 $('.popup2').magnificPopup({
 type: 'inline',
 alignTop: true,
 overflowY: 'scroll' // as we know that popup content is tall we set scroll overflow by default to avoid jump
 });
 });

 $(function () {
 var currentHash = $(location).attr('hash');
 var filename = $(location).attr('href').split('/').reverse()[1];
 $(document).scroll(function () {
 $('.header-link').each(function () {
 var top = window.pageYOffset;
 var distance = top - $(this).offset().top;
 var rawHash = $(this).attr('href');
 var contentId = rawHash.replace('#', '');
 var tocId = rawHash.replace('#', filename+'\_toc\_');
 var searchContentId = $('#'+contentId);
 var searchTocId = $('#'+tocId);

 if (distance < 50 && distance > -50 && currentHash != tocId) {
 // window.history.pushState(window.location.href, "title", currentHash); //Update browser URL history with current anchor
 if ((searchContentId.hasClass('class-h1') || searchContentId.hasClass('class-h2') || searchContentId.hasClass('class-h3')) && $('.nav\_\_items').has('#'+tocId).length) {
 // console.log(contentId);
 // console.log(tocId);
 searchTocId.prop("checked", true); //Open folded Toc menu
 $('.current\_contents').removeClass("current\_contents");

 //Check if Buttoned ToC with children
 // console.log(searchTocId);
 if( searchTocId.prop("tagName") == "INPUT" ) {
 // console.log("input detected");
 if( searchTocId.prop("name") == "sublevel" ) {
 searchTocId.closest("li.has-grandchildren").find("label > button").addClass("current\_contents");
 }
 }
 else {
 searchTocId.addClass("current\_contents");
 }

 // Open folded Toc menu
 searchTocId.closest("li.has-grandchildren").find("input").first().prop("checked", true);
 searchTocId.closest("li.has-children").find("input").first().prop("checked", true);

 currentHash = tocId;
 }
 }
 });
 });
 });

 window.onscroll = function () {
 var offset = 64 - $(window).scrollTop();
 if(offset < 0) { offset = 0; }
 $(".sidebar\_\_left").css("top", offset); //sticky sidebar
 $(".extend\_sidebar").css("top", offset);
 $(".local\_navigation").css("top", offset); //sticky local navigation
 $(".searchbox").css("top", offset); //sticky searchbox
 // console.log(offset);
 };

 var toggle = 0;

 function extendnav() {
 if (toggle == 0) {
 sidebar\_\_left.classList.add("extend\_sidebar");
 toggle = 1;
 }
 else {
 sidebar\_\_left.classList.remove("extend\_sidebar");
 toggle = 0;
 }
 }

 function copyurl(item)
 {
 // location.href = item.href;
 var copiedurl = decodeURI(item.href);
 var tempurl = $('<input>').val(copiedurl).appendTo('body').select();
 document.execCommand('copy');
 // console.log(copiedurl);
 // document.getElementById(id).href;
 // document.execCommand('copy');
 }

 function movetopage(targeturl)
 {
 location.href=targeturl;
 }

 function tab\_handler(en){
 console.log(en.textContent);
 var tabs = document.getElementById('tabs').children;
 for(i = 0; i < tabs.length; i++) {
 tabs[i].className = "tab not\_\_selected";
 $(".archive").removeClass(tabs[i].textContent);
 }
 en.className = "tab selected";
 $(".archive").addClass(en.textContent);
 }

 // When a page is reloaded, a previously selected tab and tab\_contents will be recalled to the current page keeping the same status until user's windows is closed.
 $(document).ready(function() {
 //defualt Tab Status. 
 if(sessionStorage.getItem("tabActive") == null){
 console.log("tabActive:" + sessionStorage.getItem("tabActive")); 
 $(".tab:nth-child(1)").addClass("selected");
 $(".tab\_contents:nth-of-type(1)").addClass("active");
 } else if(sessionStorage.getItem("tabActive") !== null) {
 console.log("tabActive: Not Null"); 
 $('[data-id="' + sessionStorage.getItem("tabActive")+'"]').addClass("active");
 $('[data-tab-name="'+ sessionStorage.getItem("tabActive")+'"]').addClass("tab selected");
 }
 })

 // en = enable
 function tab\_handler(en){
 // console.log(en.textContent);
 var tabs = document.getElementById('tabs').children;
 for(i = 0; i < tabs.length; i++) {
 tabs[i].className = "tab not\_\_selected";
 // $(".archive").removeClass(tabs[i].textContent);
 }
 en.className = "tab selected";
 // $(".archive").addClass(en.textContent);
 show\_contents(en.textContent);
 }

 // Controling a clicked tab's contents when a click event occurs. 
 function show\_contents(temp){
 // Seaching active class within .tab\_contents classes in the DOM tree, and remove the "active" class and set the disply as none. 
 $(".tab\_contents.active").removeClass("active");
 // in the "tab\_contents" class, searching for a matched ID to add a new class to the speidic ID. 
 $(".tab\_contents").each(function(){
 // Searching for a ID parameter "temp" and add a class, active, and css style. 
 // $("#" + temp).addClass("active");
 $('[data-id="' + temp +'"]').addClass("active");
 console.log(temp+":"+ $('.tab\_contents').hasClass("active")); 
 // To store user's data in the local storage, collect a class data of the active class.
 sessionStorage.setItem('tabActive', temp); 
 })
 }

![](/assets/images/search_icon.png)

[![](/assets/images/robotis_emanual_logo.png)](http://emanual.robotis.com "http://emanual.robotis.com")

* DYNAMIXEL
	+ P Series
		- [PH54-200-S500-R (H54P)](https://emanual.robotis.com/docs/en/dxl/p/ph54-200-s500-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph54-200-s500-r/")
		- [PH54-100-S500-R (H54P)](https://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/")
		- [PH42-020-S300-R (H42P)](https://emanual.robotis.com/docs/en/dxl/p/ph42-020-s300-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph42-020-s300-r/")
		- [PM54-060-S250-R (M54P)](https://emanual.robotis.com/docs/en/dxl/p/pm54-060-s250-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm54-060-s250-r/")
		- [PM54-040-S250-R (M54P)](https://emanual.robotis.com/docs/en/dxl/p/pm54-040-s250-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm54-040-s250-r/")
		- [PM42-010-S260-R (M42P)](https://emanual.robotis.com/docs/en/dxl/p/pm42-010-s260-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm42-010-s260-r/")
	+ DYNAMIXEL DRIVE (DYD)
		- [DYD-11](https://emanual.robotis.com/docs/en/all-dyd/dyd-11 "https://emanual.robotis.com/docs/en/all-dyd/dyd-11")
		- [DYD-14](https://emanual.robotis.com/docs/en/all-dyd/dyd-14 "https://emanual.robotis.com/docs/en/all-dyd/dyd-14")
		- [DYD-17](https://emanual.robotis.com/docs/en/all-dyd/dyd-17 "https://emanual.robotis.com/docs/en/all-dyd/dyd-17")
	+ [X Series](https://emanual.robotis.com/docs/en/dxl/x/ "https://emanual.robotis.com/docs/en/dxl/x/")
	+ XW Series
		- [XW540-T140](https://emanual.robotis.com/docs/en/dxl/x/xw540-t140/ "https://emanual.robotis.com/docs/en/dxl/x/xw540-t140/")
		- [XW540-T260](https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/ "https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/")
		- [XW430-T200](https://emanual.robotis.com/docs/en/dxl/x/xw430-t200/ "https://emanual.robotis.com/docs/en/dxl/x/xw430-t200/")
		- [XW430-T333](https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/ "https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/")
	+ XD Series
		- [XD540-T150](https://emanual.robotis.com/docs/en/dxl/x/xd540-t150/ "https://emanual.robotis.com/docs/en/dxl/x/xd540-t150/")
		- [XD540-T270](https://emanual.robotis.com/docs/en/dxl/x/xd540-t270/ "https://emanual.robotis.com/docs/en/dxl/x/xd540-t270/")
		- [XD430-T210](https://emanual.robotis.com/docs/en/dxl/x/xd430-t210/ "https://emanual.robotis.com/docs/en/dxl/x/xd430-t210/")
		- [XD430-T350](https://emanual.robotis.com/docs/en/dxl/x/xd430-t350/ "https://emanual.robotis.com/docs/en/dxl/x/xd430-t350/")
	+ XH Series
		- [XH540-W150](https://emanual.robotis.com/docs/en/dxl/x/xh540-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-w150/")
		- [XH540-W270](https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/")
		- [XH540-V150](https://emanual.robotis.com/docs/en/dxl/x/xh540-v150/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-v150/")
		- [XH540-V270](https://emanual.robotis.com/docs/en/dxl/x/xh540-v270/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-v270/")
		- [XH430-W210](https://emanual.robotis.com/docs/en/dxl/x/xh430-w210/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-w210/")
		- [XH430-W350](https://emanual.robotis.com/docs/en/dxl/x/xh430-w350/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-w350/")
		- [XH430-V210](https://emanual.robotis.com/docs/en/dxl/x/xh430-v210/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-v210/")
		- [XH430-V350](https://emanual.robotis.com/docs/en/dxl/x/xh430-v350/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-v350/")
	+ XM Series
		- [XM540-W150](https://emanual.robotis.com/docs/en/dxl/x/xm540-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xm540-w150/")
		- [XM540-W270](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/ "https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/")
		- [XM430-W210](https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/ "https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/")
		- [XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/ "https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/")
	+ XC Series
		- [2XC430-W250](https://emanual.robotis.com/docs/en/dxl/x/2xc430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/2xc430-w250/")
		- [XC430-W150](https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/")
		- [XC430-W240](https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/")
		- [XC430-T150BB](https://emanual.robotis.com/docs/en/dxl/x/xc430-t150bb/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-t150bb/")
		- [XC430-T240BB](https://emanual.robotis.com/docs/en/dxl/x/xc430-t240bb/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-t240bb/")
		- [XC330-T288](https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/")
		- [XC330-T181](https://emanual.robotis.com/docs/en/dxl/x/xc330-t181/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-t181/")
		- [XC330-M181](https://emanual.robotis.com/docs/en/dxl/x/xc330-m181/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-m181/")
		- [XC330-M288](https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/")
	+ XL Series
		- [2XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/")
		- [XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/")
		- [XL330-M077](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/ "https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/")
		- [XL330-M288](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/ "https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/")
		- [XL-320](https://emanual.robotis.com/docs/en/dxl/x/xl320/ "https://emanual.robotis.com/docs/en/dxl/x/xl320/")
	+ MX Series
		- [MX-106T/R(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/")
		- [MX-64T/R/AT/AR(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/")
		- [MX-28T/R/AT/AR(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/")
		- [MX-106T/R](https://emanual.robotis.com/docs/en/dxl/mx/mx-106/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-106/")
		- [MX-64T/R/AT/AR](https://emanual.robotis.com/docs/en/dxl/mx/mx-64/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-64/")
		- [MX-28T/R/AT/AR](https://emanual.robotis.com/docs/en/dxl/mx/mx-28/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-28/")
		- [MX-12W](https://emanual.robotis.com/docs/en/dxl/mx/mx-12w/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-12w/")
	+ AX Series
		- [AX-18F/18A](https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/")
		- [AX-12/12+/12A](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/")
		- [AX-12W](https://emanual.robotis.com/docs/en/dxl/ax/ax-12w/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-12w/")
	+ [DYNAMIXEL Protocol 1.0](https://emanual.robotis.com/docs/en/dxl/protocol1/ "https://emanual.robotis.com/docs/en/dxl/protocol1/")
	+ [DYNAMIXEL Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/ "https://emanual.robotis.com/docs/en/dxl/protocol2/")
	+ EX Series 
		- [EX-106+](https://emanual.robotis.com/docs/en/dxl/ex/ex-106+/ "https://emanual.robotis.com/docs/en/dxl/ex/ex-106+/")
	+ DX Series 
		- [DX-113](https://emanual.robotis.com/docs/en/dxl/dx/dx-113/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-113/")
		- [DX-116](https://emanual.robotis.com/docs/en/dxl/dx/dx-116/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-116/")
		- [DX-117](https://emanual.robotis.com/docs/en/dxl/dx/dx-117/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-117/")
	+ RX Series 
		- [RX-10](https://emanual.robotis.com/docs/en/dxl/rx/rx-10/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-10/")
		- [RX-24F](https://emanual.robotis.com/docs/en/dxl/rx/rx-24f/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-24f/")
		- [RX-28](https://emanual.robotis.com/docs/en/dxl/rx/rx-28/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-28/")
		- [RX-64](https://emanual.robotis.com/docs/en/dxl/rx/rx-64/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-64/")
	+ PRO Series 
		- [H54-200-S500-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-ra/")
		- [H54-100-S500-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-ra/")
		- [H42-20-S300-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-ra/")
		- [M54-60-S250-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-ra/")
		- [M54-40-S250-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-ra/")
		- [M42-10-S260-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-ra/")
		- [H54-200-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-r/")
		- [H54-100-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-r/")
		- [H42-20-S300-R](https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-r/")
		- [M54-60-S250-R](https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-r/")
		- [M54-40-S250-R](https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-r/")
		- [M42-10-S260-R](https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-r/")
		- [L54-50-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s500-r/")
		- [L54-50-S290-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s290-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s290-r/")
		- [L54-30-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s500-r/")
		- [L54-30-S400-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s400-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s400-r/")
		- [L42-10-S300-R](https://emanual.robotis.com/docs/en/dxl/pro/l42-10-s300-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l42-10-s300-r/")
* DYNAMIXEL  SYSTEM
	+ [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/")
	+ OpenMANIPULATOR
		- [OpenMANIPULATOR-P](https://emanual.robotis.com/docs/en/platform/openmanipulator_p/overview/ "https://emanual.robotis.com/docs/en/platform/openmanipulator_p/overview/")
		- [OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/ "https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/")
		- [Manipulator-H](https://emanual.robotis.com/docs/en/platform/manipulator_h/introduction/ "https://emanual.robotis.com/docs/en/platform/manipulator_h/introduction/")
	+ Robot Hands
		- [RH-P12-RN(A)](https://emanual.robotis.com/docs/en/platform/rh_p12_rna/ "https://emanual.robotis.com/docs/en/platform/rh_p12_rna/")
		- [RH-P12-RN-UR](https://emanual.robotis.com/docs/en/platform/rh_p12_rn_ur/ "https://emanual.robotis.com/docs/en/platform/rh_p12_rn_ur/")
	+ ROBOTIS OP
		- [ROBOTIS OP3](https://emanual.robotis.com/docs/en/platform/op3/introduction/ "https://emanual.robotis.com/docs/en/platform/op3/introduction/")
		- [ROBOTIS OP](https://emanual.robotis.com/docs/en/platform/op/getting_started/ "https://emanual.robotis.com/docs/en/platform/op/getting_started/")
		- [ROBOTIS OP2](https://emanual.robotis.com/docs/en/platform/op2/getting_started/ "https://emanual.robotis.com/docs/en/platform/op2/getting_started/")
	+ [THORMANG3](https://emanual.robotis.com/docs/en/platform/thormang3/introduction/ "https://emanual.robotis.com/docs/en/platform/thormang3/introduction/")
* EDUCATIONAL  KITS
	+ PLAY
		- [PLAY 300](https://emanual.robotis.com/docs/en/edu/play/play-300/ "https://emanual.robotis.com/docs/en/edu/play/play-300/")
		- [PLAY 600](https://emanual.robotis.com/docs/en/edu/play/play-600/ "https://emanual.robotis.com/docs/en/edu/play/play-600/")
		- [PLAY 700](https://emanual.robotis.com/docs/en/edu/play/play-700/ "https://emanual.robotis.com/docs/en/edu/play/play-700/")
	+ ROBOTIS DREAM II
		- [LEVEL 1](https://emanual.robotis.com/docs/en/edu/dream/dream2-1/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-1/")
		- [LEVEL 2](https://emanual.robotis.com/docs/en/edu/dream/dream2-2/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-2/")
		- [LEVEL 3](https://emanual.robotis.com/docs/en/edu/dream/dream2-3/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-3/")
		- [LEVEL 4](https://emanual.robotis.com/docs/en/edu/dream/dream2-4/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-4/")
		- [LEVEL 5](https://emanual.robotis.com/docs/en/edu/dream/dream2-5/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-5/")
		- [School Set](https://emanual.robotis.com/docs/en/edu/dream/dream2-schoolset/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-schoolset/")
	+ ROBOTIS BIOLOID
		- [ROBOTIS STEM](https://emanual.robotis.com/docs/en/edu/bioloid/stem/ "https://emanual.robotis.com/docs/en/edu/bioloid/stem/")
		- [ROBOTIS PREMIUM](https://emanual.robotis.com/docs/en/edu/bioloid/premium/ "https://emanual.robotis.com/docs/en/edu/bioloid/premium/")
		- [ROBOTIS GP](https://emanual.robotis.com/docs/en/edu/bioloid/gp/ "https://emanual.robotis.com/docs/en/edu/bioloid/gp/")
		- [Beginner Level](https://emanual.robotis.com/docs/en/edu/bioloid/beginner/ "https://emanual.robotis.com/docs/en/edu/bioloid/beginner/")
		- [Comprehensive Level](https://emanual.robotis.com/docs/en/edu/bioloid/comprehensive/ "https://emanual.robotis.com/docs/en/edu/bioloid/comprehensive/")
	+ ROBOTIS ENGINEER
		- [Kit 1](https://emanual.robotis.com/docs/en/edu/engineer/kit1/ "https://emanual.robotis.com/docs/en/edu/engineer/kit1/")
		- [Kit 2](https://emanual.robotis.com/docs/en/edu/engineer/kit2_introduction/ "https://emanual.robotis.com/docs/en/edu/engineer/kit2_introduction/")
	+ [ROBOTIS MINI](https://emanual.robotis.com/docs/en/edu/mini/ "https://emanual.robotis.com/docs/en/edu/mini/")
	+ OLLO 
		- [BUG KIT](https://emanual.robotis.com/docs/en/edu/ollo/bugkit/ "https://emanual.robotis.com/docs/en/edu/ollo/bugkit/")
		- [EXPLORER](https://emanual.robotis.com/docs/en/edu/ollo/explorer/ "https://emanual.robotis.com/docs/en/edu/ollo/explorer/")
		- [INVENTOR](https://emanual.robotis.com/docs/en/edu/ollo/inventor/ "https://emanual.robotis.com/docs/en/edu/ollo/inventor/")
	+ DREAM 
		- [LEVEL 2](https://emanual.robotis.com/docs/en/edu/dream/dream1-2/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-2/")
		- [LEVEL 3](https://emanual.robotis.com/docs/en/edu/dream/dream1-3/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-3/")
		- [LEVEL 4](https://emanual.robotis.com/docs/en/edu/dream/dream1-4/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-4/")
		- [SET A](https://emanual.robotis.com/docs/en/edu/dream/dream-a/ "https://emanual.robotis.com/docs/en/edu/dream/dream-a/")
		- [SET B](https://emanual.robotis.com/docs/en/edu/dream/dream-b/ "https://emanual.robotis.com/docs/en/edu/dream/dream-b/")
* SOFTWARE
	+ DYNAMIXEL
		- [DYNAMIXEL SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/")
		- [DYNAMIXEL Workbench](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/")
		- [DYNAMIXEL Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/")
	+ Mobile Apps
		- [ROBOTIS MINI](https://emanual.robotis.com/docs/en/software/mobile_app/mini_app/ "https://emanual.robotis.com/docs/en/software/mobile_app/mini_app/")
		- [R+ Block](https://emanual.robotis.com/docs/en/software/rplus2/rplus2_block/ "https://emanual.robotis.com/docs/en/software/rplus2/rplus2_block/")
	+ R+ 1.0
		- [R+ Task](https://emanual.robotis.com/docs/en/software/rplus1/task/getting_started/ "https://emanual.robotis.com/docs/en/software/rplus1/task/getting_started/")
		- [R+ Manager](https://emanual.robotis.com/docs/en/software/rplus1/manager/ "https://emanual.robotis.com/docs/en/software/rplus1/manager/")
		- [R+ Motion](https://emanual.robotis.com/docs/en/software/rplus1/motion/ "https://emanual.robotis.com/docs/en/software/rplus1/motion/")
		- [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/ "https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/")
	+ R+ 2.0
		- [R+ Task 2.0](https://emanual.robotis.com/docs/en/software/rplus2/task/ "https://emanual.robotis.com/docs/en/software/rplus2/task/")
		- [R+ Manager 2.0](https://emanual.robotis.com/docs/en/software/rplus2/manager/ "https://emanual.robotis.com/docs/en/software/rplus2/manager/")
		- [R+ Motion 2.0](https://emanual.robotis.com/docs/en/software/rplus2/motion/ "https://emanual.robotis.com/docs/en/software/rplus2/motion/")
		- [R+ Design 2.0](https://emanual.robotis.com/docs/en/software/rplus2/design/ "https://emanual.robotis.com/docs/en/software/rplus2/design/")
		- [R+ Scratch](https://emanual.robotis.com/docs/en/software/rplus2/scratch/ "https://emanual.robotis.com/docs/en/software/rplus2/scratch/")
	+ [R+ Task 3.0](https://emanual.robotis.com/docs/en/software/rplustask3/ "https://emanual.robotis.com/docs/en/software/rplustask3/")
	+ R+ Mobile
		- [R+ m.Task 2.0](https://emanual.robotis.com/docs/en/software/rplus_mobile/mtask20/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mtask20/")
		- [R+ m.Motion 2.0](https://emanual.robotis.com/docs/en/software/rplus_mobile/mmotion/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mmotion/")
		- [R+ m.Design](https://emanual.robotis.com/docs/en/software/rplus_mobile/mdesign/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mdesign/")
	+ EMBEDDED SDK
		- [Embedded C(CM-510/700)](https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm510/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm510/")
		- [Embedded C(CM-530)](https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/")
		- [ZIGBEE SDK](https://emanual.robotis.com/docs/en/software/embedded_sdk/zigbee_sdk/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/zigbee_sdk/")
	+ [Arduino IDE](https://emanual.robotis.com/docs/en/software/arduino_ide/ "https://emanual.robotis.com/docs/en/software/arduino_ide/")
	+ [ROBOTIS Framework Packages](https://emanual.robotis.com/docs/en/software/robotis_framework_packages/ "https://emanual.robotis.com/docs/en/software/robotis_framework_packages/")
	+ [ROBOTIS Manipulator library](https://emanual.robotis.com/docs/en/software/robotis_manipulator_libs/ "https://emanual.robotis.com/docs/en/software/robotis_manipulator_libs/")
	+ [\*OpenCM IDE](https://emanual.robotis.com/docs/en/software/opencm_ide/getting_started/ "https://emanual.robotis.com/docs/en/software/opencm_ide/getting_started/")
* PARTS
	+ Controller
		- [CM-50](https://emanual.robotis.com/docs/en/parts/controller/cm-50/ "https://emanual.robotis.com/docs/en/parts/controller/cm-50/")
		- [CM-150](https://emanual.robotis.com/docs/en/parts/controller/cm-150/ "https://emanual.robotis.com/docs/en/parts/controller/cm-150/")
		- [CM-151](https://emanual.robotis.com/docs/en/parts/controller/cm-151/ "https://emanual.robotis.com/docs/en/parts/controller/cm-151/")
		- [CM-200](https://emanual.robotis.com/docs/en/parts/controller/cm-200/ "https://emanual.robotis.com/docs/en/parts/controller/cm-200/")
		- [CM-530](https://emanual.robotis.com/docs/en/parts/controller/cm-530/ "https://emanual.robotis.com/docs/en/parts/controller/cm-530/")
		- [CM-550](https://emanual.robotis.com/docs/en/parts/controller/cm-550/ "https://emanual.robotis.com/docs/en/parts/controller/cm-550/")
		- [CM-700](https://emanual.robotis.com/docs/en/parts/controller/cm-700/ "https://emanual.robotis.com/docs/en/parts/controller/cm-700/")
		- [OpenRB-150](https://emanual.robotis.com/docs/en/parts/controller/openrb-150/ "https://emanual.robotis.com/docs/en/parts/controller/openrb-150/")
		- [OpenCM9.04](https://emanual.robotis.com/docs/en/parts/controller/opencm904/ "https://emanual.robotis.com/docs/en/parts/controller/opencm904/")
		- [OpenCM 485 EXP](https://emanual.robotis.com/docs/en/parts/controller/opencm485exp/ "https://emanual.robotis.com/docs/en/parts/controller/opencm485exp/")
		- [OpenCR1.0](https://emanual.robotis.com/docs/en/parts/controller/opencr10/ "https://emanual.robotis.com/docs/en/parts/controller/opencr10/")
		- [CM-100A](https://emanual.robotis.com/docs/en/parts/controller/cm-100/ "https://emanual.robotis.com/docs/en/parts/controller/cm-100/")
		- [CM-5](https://emanual.robotis.com/docs/en/parts/controller/cm-5/ "https://emanual.robotis.com/docs/en/parts/controller/cm-5/")
		- [CM-510](https://emanual.robotis.com/docs/en/parts/controller/cm-510/ "https://emanual.robotis.com/docs/en/parts/controller/cm-510/")
		- [CM-900](https://emanual.robotis.com/docs/en/parts/controller/cm-900/ "https://emanual.robotis.com/docs/en/parts/controller/cm-900/")
	+ Communication
		- [RC-100A/100B](https://emanual.robotis.com/docs/en/parts/communication/rc-100/ "https://emanual.robotis.com/docs/en/parts/communication/rc-100/")
		- [RC-200](https://emanual.robotis.com/docs/en/parts/communication/rc-200/ "https://emanual.robotis.com/docs/en/parts/communication/rc-200/")
		- [BT-210](https://emanual.robotis.com/docs/en/parts/communication/bt-210/ "https://emanual.robotis.com/docs/en/parts/communication/bt-210/")
		- [BT-410](https://emanual.robotis.com/docs/en/parts/communication/bt-410/ "https://emanual.robotis.com/docs/en/parts/communication/bt-410/")
		- [BT-410 Dongle](https://emanual.robotis.com/docs/en/parts/communication/bt-410-dongle/ "https://emanual.robotis.com/docs/en/parts/communication/bt-410-dongle/")
		- [ZIG-100/110A](https://emanual.robotis.com/docs/en/parts/communication/zig-110/ "https://emanual.robotis.com/docs/en/parts/communication/zig-110/")
		- [BT-100/110A](https://emanual.robotis.com/docs/en/parts/communication/bt-110/ "https://emanual.robotis.com/docs/en/parts/communication/bt-110/")
		- [ZIG2Serial](https://emanual.robotis.com/docs/en/parts/communication/zig2serial/ "https://emanual.robotis.com/docs/en/parts/communication/zig2serial/")
	+ Motors
		- [Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/gm-10a/ "https://emanual.robotis.com/docs/en/parts/motor/gm-10a/")
		- [Servo Motor](https://emanual.robotis.com/docs/en/parts/motor/servo_motor/ "https://emanual.robotis.com/docs/en/parts/motor/servo_motor/")
		- [High Speed Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/h_speed_geared_motor/ "https://emanual.robotis.com/docs/en/parts/motor/h_speed_geared_motor/")
		- [Low Speed Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/l_speed_geared_motor/ "https://emanual.robotis.com/docs/en/parts/motor/l_speed_geared_motor/")
	+ Interface
		- [DYNAMIXEL Communication Bridge](https://emanual.robotis.com/docs/en/parts/interface/dxl_bridge/ "https://emanual.robotis.com/docs/en/parts/interface/dxl_bridge/")
		- [LN-101](https://emanual.robotis.com/docs/en/parts/interface/ln-101/ "https://emanual.robotis.com/docs/en/parts/interface/ln-101/")
		- [U2D2](https://emanual.robotis.com/docs/en/parts/interface/u2d2/ "https://emanual.robotis.com/docs/en/parts/interface/u2d2/")
		- [U2D2 Power Hub](https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/ "https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/")
		- [DYNAMIXEL Shield](https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/ "https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/")
		- [DYNAMIXEL Shield MKR](https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/ "https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/")
		- [USB2DYNAMIXEL](https://emanual.robotis.com/docs/en/parts/interface/usb2dynamixel/ "https://emanual.robotis.com/docs/en/parts/interface/usb2dynamixel/")
	+ Sensors
		- [IR Sensor](https://emanual.robotis.com/docs/en/parts/sensor/irss-10/ "https://emanual.robotis.com/docs/en/parts/sensor/irss-10/")
		- [Distance Sensor](https://emanual.robotis.com/docs/en/parts/sensor/dms-80/ "https://emanual.robotis.com/docs/en/parts/sensor/dms-80/")
		- [Touch Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ts-10/ "https://emanual.robotis.com/docs/en/parts/sensor/ts-10/")
		- [Gyro Sensor](https://emanual.robotis.com/docs/en/parts/sensor/gs-12/ "https://emanual.robotis.com/docs/en/parts/sensor/gs-12/")
		- [IR Array Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ir-array/ "https://emanual.robotis.com/docs/en/parts/sensor/ir-array/")
		- [Color Sensor](https://emanual.robotis.com/docs/en/parts/sensor/cs-10/ "https://emanual.robotis.com/docs/en/parts/sensor/cs-10/")
		- [Magnetic Sensor](https://emanual.robotis.com/docs/en/parts/sensor/mgss-10/ "https://emanual.robotis.com/docs/en/parts/sensor/mgss-10/")
		- [Temperature Sensor](https://emanual.robotis.com/docs/en/parts/sensor/tps-10/ "https://emanual.robotis.com/docs/en/parts/sensor/tps-10/")
		- [Motion Sensor](https://emanual.robotis.com/docs/en/parts/sensor/pir-10/ "https://emanual.robotis.com/docs/en/parts/sensor/pir-10/")
		- [Integrated Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ax-s1/ "https://emanual.robotis.com/docs/en/parts/sensor/ax-s1/")
	+ Display
		- [LED Module](https://emanual.robotis.com/docs/en/parts/display/lm-10/ "https://emanual.robotis.com/docs/en/parts/display/lm-10/")
* FAQ
	+ [DYNAMIXEL Selection Guide](https://emanual.robotis.com/docs/en/reference/dxl-selection-guide/ "https://emanual.robotis.com/docs/en/reference/dxl-selection-guide/")
	+ [DYNAMIXEL Quick Start Guide](https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-guide/ "https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-guide/")
	+ [DYNAMIXEL](https://emanual.robotis.com/docs/en/faq/faq_dynamixel/ "https://emanual.robotis.com/docs/en/faq/faq_dynamixel/")
	+ [DYNAMIXEL SYSTEM](https://emanual.robotis.com/docs/en/faq/faq_platform/ "https://emanual.robotis.com/docs/en/faq/faq_platform/")
	+ [EDUCATION KITS](https://emanual.robotis.com/docs/en/faq/faq_steam/ "https://emanual.robotis.com/docs/en/faq/faq_steam/")
	+ [SOFTWARE](https://emanual.robotis.com/docs/en/faq/faq_software/ "https://emanual.robotis.com/docs/en/faq/faq_software/")
	+ [PARTS](https://emanual.robotis.com/docs/en/faq/faq_parts/ "https://emanual.robotis.com/docs/en/faq/faq_parts/")
	+ [GENERAL](https://emanual.robotis.com/docs/en/faq/general "https://emanual.robotis.com/docs/en/faq/general")

 DYNAMIXEL

[P Series](https://emanual.robotis.com/docs/en/dxl/p/ "https://emanual.robotis.com/docs/en/dxl/p/")

[PH54-200-S500-R (H54P)](https://emanual.robotis.com/docs/en/dxl/p/ph54-200-s500-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph54-200-s500-r/")
[PH54-100-S500-R (H54P)](https://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/")
[PH42-020-S300-R (H42P)](https://emanual.robotis.com/docs/en/dxl/p/ph42-020-s300-r/ "https://emanual.robotis.com/docs/en/dxl/p/ph42-020-s300-r/")
[PM54-060-S250-R (M54P)](https://emanual.robotis.com/docs/en/dxl/p/pm54-060-s250-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm54-060-s250-r/")
[PM54-040-S250-R (M54P)](https://emanual.robotis.com/docs/en/dxl/p/pm54-040-s250-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm54-040-s250-r/")
[PM42-010-S260-R (M42P)](https://emanual.robotis.com/docs/en/dxl/p/pm42-010-s260-r/ "https://emanual.robotis.com/docs/en/dxl/p/pm42-010-s260-r/")

[DYNAMIXEL DRIVE (DYD)](https://emanual.robotis.com/docs/en/all-dyd/ "https://emanual.robotis.com/docs/en/all-dyd/")

[DYD-11](https://emanual.robotis.com/docs/en/all-dyd/dyd-11 "https://emanual.robotis.com/docs/en/all-dyd/dyd-11")
[DYD-14](https://emanual.robotis.com/docs/en/all-dyd/dyd-14 "https://emanual.robotis.com/docs/en/all-dyd/dyd-14")
[DYD-17](https://emanual.robotis.com/docs/en/all-dyd/dyd-17 "https://emanual.robotis.com/docs/en/all-dyd/dyd-17")

[X Series](https://emanual.robotis.com/docs/en/dxl/x/ "https://emanual.robotis.com/docs/en/dxl/x/")

XW Series

[XW540-T140](https://emanual.robotis.com/docs/en/dxl/x/xw540-t140/ "https://emanual.robotis.com/docs/en/dxl/x/xw540-t140/")
[XW540-T260](https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/ "https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/")
[XW430-T200](https://emanual.robotis.com/docs/en/dxl/x/xw430-t200/ "https://emanual.robotis.com/docs/en/dxl/x/xw430-t200/")
[XW430-T333](https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/ "https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/")

XD Series

[XD540-T150](https://emanual.robotis.com/docs/en/dxl/x/xd540-t150/ "https://emanual.robotis.com/docs/en/dxl/x/xd540-t150/")
[XD540-T270](https://emanual.robotis.com/docs/en/dxl/x/xd540-t270/ "https://emanual.robotis.com/docs/en/dxl/x/xd540-t270/")
[XD430-T210](https://emanual.robotis.com/docs/en/dxl/x/xd430-t210/ "https://emanual.robotis.com/docs/en/dxl/x/xd430-t210/")
[XD430-T350](https://emanual.robotis.com/docs/en/dxl/x/xd430-t350/ "https://emanual.robotis.com/docs/en/dxl/x/xd430-t350/")

XH Series

[XH540-W150](https://emanual.robotis.com/docs/en/dxl/x/xh540-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-w150/")
[XH540-W270](https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/")
[XH540-V150](https://emanual.robotis.com/docs/en/dxl/x/xh540-v150/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-v150/")
[XH540-V270](https://emanual.robotis.com/docs/en/dxl/x/xh540-v270/ "https://emanual.robotis.com/docs/en/dxl/x/xh540-v270/")
[XH430-W210](https://emanual.robotis.com/docs/en/dxl/x/xh430-w210/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-w210/")
[XH430-W350](https://emanual.robotis.com/docs/en/dxl/x/xh430-w350/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-w350/")
[XH430-V210](https://emanual.robotis.com/docs/en/dxl/x/xh430-v210/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-v210/")
[XH430-V350](https://emanual.robotis.com/docs/en/dxl/x/xh430-v350/ "https://emanual.robotis.com/docs/en/dxl/x/xh430-v350/")

XM Series

[XM540-W150](https://emanual.robotis.com/docs/en/dxl/x/xm540-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xm540-w150/")
[XM540-W270](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/ "https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/")
[XM430-W210](https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/ "https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/")
[XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/ "https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/")

XC Series

[2XC430-W250](https://emanual.robotis.com/docs/en/dxl/x/2xc430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/2xc430-w250/")
[XC430-W150](https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-w150/")
[XC430-W240](https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/")
[XC430-T150BB](https://emanual.robotis.com/docs/en/dxl/x/xc430-t150bb/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-t150bb/")
[XC430-T240BB](https://emanual.robotis.com/docs/en/dxl/x/xc430-t240bb/ "https://emanual.robotis.com/docs/en/dxl/x/xc430-t240bb/")
[XC330-T288](https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/")
[XC330-T181](https://emanual.robotis.com/docs/en/dxl/x/xc330-t181/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-t181/")
[XC330-M181](https://emanual.robotis.com/docs/en/dxl/x/xc330-m181/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-m181/")
[XC330-M288](https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/ "https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/")

XL Series

[2XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/")
[XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/ "https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/")
[XL330-M077](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/ "https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/")
[XL330-M288](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/ "https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/")
[XL-320](https://emanual.robotis.com/docs/en/dxl/x/xl320/ "https://emanual.robotis.com/docs/en/dxl/x/xl320/")

[MX Series](https://emanual.robotis.com/docs/en/dxl/mx/ "https://emanual.robotis.com/docs/en/dxl/mx/")

[MX-106T/R(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/")
[MX-64T/R/AT/AR(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/")
[MX-28T/R/AT/AR(2.0)](https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/")
[MX-106T/R](https://emanual.robotis.com/docs/en/dxl/mx/mx-106/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-106/")
[MX-64T/R/AT/AR](https://emanual.robotis.com/docs/en/dxl/mx/mx-64/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-64/")
[MX-28T/R/AT/AR](https://emanual.robotis.com/docs/en/dxl/mx/mx-28/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-28/")
[MX-12W](https://emanual.robotis.com/docs/en/dxl/mx/mx-12w/ "https://emanual.robotis.com/docs/en/dxl/mx/mx-12w/")

AX Series

[AX-18F/18A](https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/")
[AX-12/12+/12A](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/")
[AX-12W](https://emanual.robotis.com/docs/en/dxl/ax/ax-12w/ "https://emanual.robotis.com/docs/en/dxl/ax/ax-12w/")

[DYNAMIXEL Protocol 1.0](https://emanual.robotis.com/docs/en/dxl/protocol1/ "https://emanual.robotis.com/docs/en/dxl/protocol1/")

[DYNAMIXEL Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/ "https://emanual.robotis.com/docs/en/dxl/protocol2/")

EX Series

[EX-106+](https://emanual.robotis.com/docs/en/dxl/ex/ex-106+/ "https://emanual.robotis.com/docs/en/dxl/ex/ex-106+/") 

DX Series

[DX-113](https://emanual.robotis.com/docs/en/dxl/dx/dx-113/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-113/") 
[DX-116](https://emanual.robotis.com/docs/en/dxl/dx/dx-116/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-116/") 
[DX-117](https://emanual.robotis.com/docs/en/dxl/dx/dx-117/ "https://emanual.robotis.com/docs/en/dxl/dx/dx-117/") 

RX Series

[RX-10](https://emanual.robotis.com/docs/en/dxl/rx/rx-10/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-10/") 
[RX-24F](https://emanual.robotis.com/docs/en/dxl/rx/rx-24f/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-24f/") 
[RX-28](https://emanual.robotis.com/docs/en/dxl/rx/rx-28/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-28/") 
[RX-64](https://emanual.robotis.com/docs/en/dxl/rx/rx-64/ "https://emanual.robotis.com/docs/en/dxl/rx/rx-64/") 

[PRO Series](https://emanual.robotis.com/docs/en/dxl/pro/ "https://emanual.robotis.com/docs/en/dxl/pro/") 

[H54-200-S500-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-ra/") 
[H54-100-S500-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-ra/") 
[H42-20-S300-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-ra/") 
[M54-60-S250-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-ra/") 
[M54-40-S250-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-ra/") 
[M42-10-S260-R(A)](https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-ra/ "https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-ra/") 
[H54-200-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-r/") 
[H54-100-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h54-100-s500-r/") 
[H42-20-S300-R](https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-r/ "https://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-r/") 
[M54-60-S250-R](https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-60-s250-r/") 
[M54-40-S250-R](https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m54-40-s250-r/") 
[M42-10-S260-R](https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-r/ "https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-r/") 
[L54-50-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s500-r/") 
[L54-50-S290-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s290-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-50-s290-r/") 
[L54-30-S500-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s500-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s500-r/") 
[L54-30-S400-R](https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s400-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l54-30-s400-r/") 
[L42-10-S300-R](https://emanual.robotis.com/docs/en/dxl/pro/l42-10-s300-r/ "https://emanual.robotis.com/docs/en/dxl/pro/l42-10-s300-r/") 

 DYNAMIXEL  SYSTEM

[TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/")

[OpenMANIPULATOR](https://emanual.robotis.com/docs/en/platform/openmanipulator_main "https://emanual.robotis.com/docs/en/platform/openmanipulator_main")

[OpenMANIPULATOR-P](https://emanual.robotis.com/docs/en/platform/openmanipulator_p/overview/ "https://emanual.robotis.com/docs/en/platform/openmanipulator_p/overview/")
[OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/ "https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/")
[Manipulator-H](https://emanual.robotis.com/docs/en/platform/manipulator_h/introduction/ "https://emanual.robotis.com/docs/en/platform/manipulator_h/introduction/") 

Robot Hands

[RH-P12-RN(A)](https://emanual.robotis.com/docs/en/platform/rh_p12_rna/ "https://emanual.robotis.com/docs/en/platform/rh_p12_rna/")
[RH-P12-RN-UR](https://emanual.robotis.com/docs/en/platform/rh_p12_rn_ur/ "https://emanual.robotis.com/docs/en/platform/rh_p12_rn_ur/")

[ROBOTIS OP](https://emanual.robotis.com/docs/en/platform/op/getting_started/ "https://emanual.robotis.com/docs/en/platform/op/getting_started/")

[ROBOTIS OP3](https://emanual.robotis.com/docs/en/platform/op3/introduction/ "https://emanual.robotis.com/docs/en/platform/op3/introduction/")
[ROBOTIS OP](https://emanual.robotis.com/docs/en/platform/op/getting_started/ "https://emanual.robotis.com/docs/en/platform/op/getting_started/") 
[ROBOTIS OP2](https://emanual.robotis.com/docs/en/platform/op2/getting_started/ "https://emanual.robotis.com/docs/en/platform/op2/getting_started/") 

[THORMANG3](https://emanual.robotis.com/docs/en/platform/thormang3/introduction/ "https://emanual.robotis.com/docs/en/platform/thormang3/introduction/")

 EDUCATIONAL  KITS

PLAY

[PLAY 300](https://emanual.robotis.com/docs/en/edu/play/play-300/ "https://emanual.robotis.com/docs/en/edu/play/play-300/")
[PLAY 600](https://emanual.robotis.com/docs/en/edu/play/play-600/ "https://emanual.robotis.com/docs/en/edu/play/play-600/")
[PLAY 700](https://emanual.robotis.com/docs/en/edu/play/play-700/ "https://emanual.robotis.com/docs/en/edu/play/play-700/")

ROBOTIS DREAM II

[LEVEL 1](https://emanual.robotis.com/docs/en/edu/dream/dream2-1/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-1/")
[LEVEL 2](https://emanual.robotis.com/docs/en/edu/dream/dream2-2/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-2/")
[LEVEL 3](https://emanual.robotis.com/docs/en/edu/dream/dream2-3/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-3/")
[LEVEL 4](https://emanual.robotis.com/docs/en/edu/dream/dream2-4/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-4/")
[LEVEL 5](https://emanual.robotis.com/docs/en/edu/dream/dream2-5/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-5/")
[School Set](https://emanual.robotis.com/docs/en/edu/dream/dream2-schoolset/ "https://emanual.robotis.com/docs/en/edu/dream/dream2-schoolset/")

[ROBOTIS BIOLOID](https://emanual.robotis.com/docs/en/edu/bioloid/ "https://emanual.robotis.com/docs/en/edu/bioloid/")

[ROBOTIS STEM](https://emanual.robotis.com/docs/en/edu/bioloid/stem/ "https://emanual.robotis.com/docs/en/edu/bioloid/stem/")
[ROBOTIS PREMIUM](https://emanual.robotis.com/docs/en/edu/bioloid/premium/ "https://emanual.robotis.com/docs/en/edu/bioloid/premium/")
[ROBOTIS GP](https://emanual.robotis.com/docs/en/edu/bioloid/gp/ "https://emanual.robotis.com/docs/en/edu/bioloid/gp/")
[Beginner Level](https://emanual.robotis.com/docs/en/edu/bioloid/beginner/ "https://emanual.robotis.com/docs/en/edu/bioloid/beginner/") 
[Comprehensive Level](https://emanual.robotis.com/docs/en/edu/bioloid/comprehensive/ "https://emanual.robotis.com/docs/en/edu/bioloid/comprehensive/") 

ROBOTIS ENGINEER

[Kit 1](https://emanual.robotis.com/docs/en/edu/engineer/kit1/ "https://emanual.robotis.com/docs/en/edu/engineer/kit1/")
[Kit 2](https://emanual.robotis.com/docs/en/edu/engineer/kit2_introduction/ "https://emanual.robotis.com/docs/en/edu/engineer/kit2_introduction/")

[ROBOTIS MINI](https://emanual.robotis.com/docs/en/edu/mini/ "https://emanual.robotis.com/docs/en/edu/mini/")

[OLLO](https://emanual.robotis.com/docs/en/edu/ollo/ollo-4/ "https://emanual.robotis.com/docs/en/edu/ollo/ollo-4/") 

[BUG KIT](https://emanual.robotis.com/docs/en/edu/ollo/bugkit/ "https://emanual.robotis.com/docs/en/edu/ollo/bugkit/") 
[EXPLORER](https://emanual.robotis.com/docs/en/edu/ollo/explorer/ "https://emanual.robotis.com/docs/en/edu/ollo/explorer/") 
[INVENTOR](https://emanual.robotis.com/docs/en/edu/ollo/inventor/ "https://emanual.robotis.com/docs/en/edu/ollo/inventor/") 

DREAM

[LEVEL 2](https://emanual.robotis.com/docs/en/edu/dream/dream1-2/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-2/") 
[LEVEL 3](https://emanual.robotis.com/docs/en/edu/dream/dream1-3/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-3/") 
[LEVEL 4](https://emanual.robotis.com/docs/en/edu/dream/dream1-4/ "https://emanual.robotis.com/docs/en/edu/dream/dream1-4/") 
[SET A](https://emanual.robotis.com/docs/en/edu/dream/dream-a/ "https://emanual.robotis.com/docs/en/edu/dream/dream-a/") 
[SET B](https://emanual.robotis.com/docs/en/edu/dream/dream-b/ "https://emanual.robotis.com/docs/en/edu/dream/dream-b/") 

 SOFTWARE

DYNAMIXEL

[DYNAMIXEL SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/")
[DYNAMIXEL Workbench](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/")
[DYNAMIXEL Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/ "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/")

Mobile Apps

[ROBOTIS MINI](https://emanual.robotis.com/docs/en/software/mobile_app/mini_app/ "https://emanual.robotis.com/docs/en/software/mobile_app/mini_app/")
[R+ Block](https://emanual.robotis.com/docs/en/software/rplus2/rplus2_block/ "https://emanual.robotis.com/docs/en/software/rplus2/rplus2_block/")

R+ 1.0

[R+ Task](https://emanual.robotis.com/docs/en/software/rplus1/task/getting_started/ "https://emanual.robotis.com/docs/en/software/rplus1/task/getting_started/")
[R+ Manager](https://emanual.robotis.com/docs/en/software/rplus1/manager/ "https://emanual.robotis.com/docs/en/software/rplus1/manager/")
[R+ Motion](https://emanual.robotis.com/docs/en/software/rplus1/motion/ "https://emanual.robotis.com/docs/en/software/rplus1/motion/")
[Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/ "https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/")

R+ 2.0

[R+ Task 2.0](https://emanual.robotis.com/docs/en/software/rplus2/task/ "https://emanual.robotis.com/docs/en/software/rplus2/task/")
[R+ Manager 2.0](https://emanual.robotis.com/docs/en/software/rplus2/manager/ "https://emanual.robotis.com/docs/en/software/rplus2/manager/")
[R+ Motion 2.0](https://emanual.robotis.com/docs/en/software/rplus2/motion/ "https://emanual.robotis.com/docs/en/software/rplus2/motion/")
[R+ Design 2.0](https://emanual.robotis.com/docs/en/software/rplus2/design/ "https://emanual.robotis.com/docs/en/software/rplus2/design/")
[R+ Scratch](https://emanual.robotis.com/docs/en/software/rplus2/scratch/ "https://emanual.robotis.com/docs/en/software/rplus2/scratch/")

[R+ Task 3.0](https://emanual.robotis.com/docs/en/software/rplustask3/ "https://emanual.robotis.com/docs/en/software/rplustask3/")

R+ Mobile

[R+ m.Task 2.0](https://emanual.robotis.com/docs/en/software/rplus_mobile/mtask20/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mtask20/")
[R+ m.Motion 2.0](https://emanual.robotis.com/docs/en/software/rplus_mobile/mmotion/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mmotion/")
[R+ m.Design](https://emanual.robotis.com/docs/en/software/rplus_mobile/mdesign/ "https://emanual.robotis.com/docs/en/software/rplus_mobile/mdesign/")

[EMBEDDED SDK](https://emanual.robotis.com/docs/en/software/embedded_sdk/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/")

[Embedded C(CM-510/700)](https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm510/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm510/")
[Embedded C(CM-530)](https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/")
[ZIGBEE SDK](https://emanual.robotis.com/docs/en/software/embedded_sdk/zigbee_sdk/ "https://emanual.robotis.com/docs/en/software/embedded_sdk/zigbee_sdk/")

[Arduino IDE](https://emanual.robotis.com/docs/en/software/arduino_ide/ "https://emanual.robotis.com/docs/en/software/arduino_ide/")

[ROBOTIS Framework Packages](https://emanual.robotis.com/docs/en/software/robotis_framework_packages/ "https://emanual.robotis.com/docs/en/software/robotis_framework_packages/")

[ROBOTIS Manipulator library](https://emanual.robotis.com/docs/en/software/robotis_manipulator_libs/ "https://emanual.robotis.com/docs/en/software/robotis_manipulator_libs/")

[OpenCM IDE](https://emanual.robotis.com/docs/en/software/opencm_ide/getting_started/ "https://emanual.robotis.com/docs/en/software/opencm_ide/getting_started/") 

 PARTS

[Controller](https://emanual.robotis.com/docs/en/parts/controller/controller_compatibility/ "https://emanual.robotis.com/docs/en/parts/controller/controller_compatibility/")

[CM-50](https://emanual.robotis.com/docs/en/parts/controller/cm-50/ "https://emanual.robotis.com/docs/en/parts/controller/cm-50/")
[CM-150](https://emanual.robotis.com/docs/en/parts/controller/cm-150/ "https://emanual.robotis.com/docs/en/parts/controller/cm-150/")
[CM-151](https://emanual.robotis.com/docs/en/parts/controller/cm-151/ "https://emanual.robotis.com/docs/en/parts/controller/cm-151/")
[CM-200](https://emanual.robotis.com/docs/en/parts/controller/cm-200/ "https://emanual.robotis.com/docs/en/parts/controller/cm-200/")
[CM-530](https://emanual.robotis.com/docs/en/parts/controller/cm-530/ "https://emanual.robotis.com/docs/en/parts/controller/cm-530/")
[CM-550](https://emanual.robotis.com/docs/en/parts/controller/cm-550/ "https://emanual.robotis.com/docs/en/parts/controller/cm-550/")
[CM-700](https://emanual.robotis.com/docs/en/parts/controller/cm-700/ "https://emanual.robotis.com/docs/en/parts/controller/cm-700/")
[OpenRB-150](https://emanual.robotis.com/docs/en/parts/controller/openrb-150/ "https://emanual.robotis.com/docs/en/parts/controller/openrb-150/")
[OpenCM9.04](https://emanual.robotis.com/docs/en/parts/controller/opencm904/ "https://emanual.robotis.com/docs/en/parts/controller/opencm904/")
[OpenCM 485 EXP](https://emanual.robotis.com/docs/en/parts/controller/opencm485exp/ "https://emanual.robotis.com/docs/en/parts/controller/opencm485exp/")
[OpenCR1.0](https://emanual.robotis.com/docs/en/parts/controller/opencr10/ "https://emanual.robotis.com/docs/en/parts/controller/opencr10/")
[CM-100A](https://emanual.robotis.com/docs/en/parts/controller/cm-100/ "https://emanual.robotis.com/docs/en/parts/controller/cm-100/") 
[CM-5](https://emanual.robotis.com/docs/en/parts/controller/cm-5/ "https://emanual.robotis.com/docs/en/parts/controller/cm-5/") 
[CM-510](https://emanual.robotis.com/docs/en/parts/controller/cm-510/ "https://emanual.robotis.com/docs/en/parts/controller/cm-510/") 
[CM-900](https://emanual.robotis.com/docs/en/parts/controller/cm-900/ "https://emanual.robotis.com/docs/en/parts/controller/cm-900/") 

Communication

[RC-100A/100B](https://emanual.robotis.com/docs/en/parts/communication/rc-100/ "https://emanual.robotis.com/docs/en/parts/communication/rc-100/")
[RC-200](https://emanual.robotis.com/docs/en/parts/communication/rc-200/ "https://emanual.robotis.com/docs/en/parts/communication/rc-200/")
[BT-210](https://emanual.robotis.com/docs/en/parts/communication/bt-210/ "https://emanual.robotis.com/docs/en/parts/communication/bt-210/")
[BT-410](https://emanual.robotis.com/docs/en/parts/communication/bt-410/ "https://emanual.robotis.com/docs/en/parts/communication/bt-410/")
[BT-410 Dongle](https://emanual.robotis.com/docs/en/parts/communication/bt-410-dongle/ "https://emanual.robotis.com/docs/en/parts/communication/bt-410-dongle/")
[ZIG-100/110A](https://emanual.robotis.com/docs/en/parts/communication/zig-110/ "https://emanual.robotis.com/docs/en/parts/communication/zig-110/") 
[BT-100/110A](https://emanual.robotis.com/docs/en/parts/communication/bt-110/ "https://emanual.robotis.com/docs/en/parts/communication/bt-110/") 
[ZIG2Serial](https://emanual.robotis.com/docs/en/parts/communication/zig2serial/ "https://emanual.robotis.com/docs/en/parts/communication/zig2serial/") 

Motors

[Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/gm-10a/ "https://emanual.robotis.com/docs/en/parts/motor/gm-10a/")
[Servo Motor](https://emanual.robotis.com/docs/en/parts/motor/servo_motor/ "https://emanual.robotis.com/docs/en/parts/motor/servo_motor/")
[High Speed Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/h_speed_geared_motor/ "https://emanual.robotis.com/docs/en/parts/motor/h_speed_geared_motor/") 
[Low Speed Geared Motor](https://emanual.robotis.com/docs/en/parts/motor/l_speed_geared_motor/ "https://emanual.robotis.com/docs/en/parts/motor/l_speed_geared_motor/") 

Interface

[DYNAMIXEL Communication Bridge](https://emanual.robotis.com/docs/en/parts/interface/dxl_bridge/ "https://emanual.robotis.com/docs/en/parts/interface/dxl_bridge/")
[LN-101](https://emanual.robotis.com/docs/en/parts/interface/ln-101/ "https://emanual.robotis.com/docs/en/parts/interface/ln-101/")
[U2D2](https://emanual.robotis.com/docs/en/parts/interface/u2d2/ "https://emanual.robotis.com/docs/en/parts/interface/u2d2/")
[U2D2 Power Hub](https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/ "https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/")
[DYNAMIXEL Shield](https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/ "https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/")
[DYNAMIXEL Shield MKR](https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/ "https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/")
[USB2DYNAMIXEL](https://emanual.robotis.com/docs/en/parts/interface/usb2dynamixel/ "https://emanual.robotis.com/docs/en/parts/interface/usb2dynamixel/") 

Sensors

[IR Sensor](https://emanual.robotis.com/docs/en/parts/sensor/irss-10/ "https://emanual.robotis.com/docs/en/parts/sensor/irss-10/")
[Distance Sensor](https://emanual.robotis.com/docs/en/parts/sensor/dms-80/ "https://emanual.robotis.com/docs/en/parts/sensor/dms-80/")
[Touch Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ts-10/ "https://emanual.robotis.com/docs/en/parts/sensor/ts-10/")
[Gyro Sensor](https://emanual.robotis.com/docs/en/parts/sensor/gs-12/ "https://emanual.robotis.com/docs/en/parts/sensor/gs-12/")
[IR Array Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ir-array/ "https://emanual.robotis.com/docs/en/parts/sensor/ir-array/")
[Color Sensor](https://emanual.robotis.com/docs/en/parts/sensor/cs-10/ "https://emanual.robotis.com/docs/en/parts/sensor/cs-10/")
[Magnetic Sensor](https://emanual.robotis.com/docs/en/parts/sensor/mgss-10/ "https://emanual.robotis.com/docs/en/parts/sensor/mgss-10/")
[Temperature Sensor](https://emanual.robotis.com/docs/en/parts/sensor/tps-10/ "https://emanual.robotis.com/docs/en/parts/sensor/tps-10/")
[Motion Sensor](https://emanual.robotis.com/docs/en/parts/sensor/pir-10/ "https://emanual.robotis.com/docs/en/parts/sensor/pir-10/")
[Integrated Sensor](https://emanual.robotis.com/docs/en/parts/sensor/ax-s1/ "https://emanual.robotis.com/docs/en/parts/sensor/ax-s1/") 

Display

[LED Module](https://emanual.robotis.com/docs/en/parts/display/lm-10/ "https://emanual.robotis.com/docs/en/parts/display/lm-10/")

 FAQ

[DYNAMIXEL Selection Guide](https://emanual.robotis.com/docs/en/reference/dxl-selection-guide/ "https://emanual.robotis.com/docs/en/reference/dxl-selection-guide/")

[DYNAMIXEL Quick Start Guide](https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-guide/ "https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-guide/")

[DYNAMIXEL](https://emanual.robotis.com/docs/en/faq/faq_dynamixel/ "https://emanual.robotis.com/docs/en/faq/faq_dynamixel/")

[DYNAMIXEL SYSTEM](https://emanual.robotis.com/docs/en/faq/faq_platform/ "https://emanual.robotis.com/docs/en/faq/faq_platform/")

[EDUCATION KITS](https://emanual.robotis.com/docs/en/faq/faq_steam/ "https://emanual.robotis.com/docs/en/faq/faq_steam/")

[SOFTWARE](https://emanual.robotis.com/docs/en/faq/faq_software/ "https://emanual.robotis.com/docs/en/faq/faq_software/")

[PARTS](https://emanual.robotis.com/docs/en/faq/faq_parts/ "https://emanual.robotis.com/docs/en/faq/faq_parts/")

[GENERAL](https://emanual.robotis.com/docs/en/faq/general "https://emanual.robotis.com/docs/en/faq/general")

![](https://www.google.com/images/cleardot.gif)

 function googleTranslateElementInit() {
 new google.translate.TranslateElement({pageLanguage: 'en', includedLanguages: 'ar,de,es,fr,hi,ja,ko,pt,ru,sv,th,tr,vi,zh-CN', layout: google.translate.TranslateElement.InlineLayout.SIMPLE, multilanguagePage: true, gaTrack: true, gaId: 'UA-83196341'}, 'google\_translate\_element');
 }

N/A

[![](/assets/images/youtube_logo.png)Youtube](https://www.youtube.com/channel/UCDCsF6xgpeb5dAmTK9Usn_g?view_as=subscriber "https://www.youtube.com/channel/UCDCsF6xgpeb5dAmTK9Usn_g?view_as=subscriber")
[![](/assets/images/shop_icon_36.png)  ROBOTIS](http://en.robotis.com "http://en.robotis.com") 
[![](/assets/images/community.jpeg)  Community](https://community.robotis.us/ "https://community.robotis.us/") 
[![](/assets/images/github_logo.png)  GitHub](https://github.com/ROBOTIS-GIT "https://github.com/ROBOTIS-GIT")

[![](/assets/images/youtube_logo.png)](https://www.youtube.com/channel/UCDCsF6xgpeb5dAmTK9Usn_g?view_as=subscriber "https://www.youtube.com/channel/UCDCsF6xgpeb5dAmTK9Usn_g?view_as=subscriber")
[![](/assets/images/shop_icon_28.png)](http://en.robotis.com "http://en.robotis.com")
[![](/assets/images/community.jpeg)](https://community.robotis.us/ "https://community.robotis.us/")
[![](/assets/images/github_logo.png)](https://github.com/ROBOTIS-GIT "https://github.com/ROBOTIS-GIT")

![](/assets/images/search_icon.png)

ToC
[▲  
TOP](# "#")

* [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/")
* 1. Overview
	+ [1. 1. Notices](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#notices "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#notices")
	+ 1. 2. Events
		- [- Online Competition on RDS](https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#online-competition-on-rds "https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#online-competition-on-rds")
		- [- Offline Competition](https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#offline-competition "https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#offline-competition")
		- [- AutoRace RBIZ Challenge](https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#autorace-rbiz-challenge "https://emanual.robotis.com/docs/en/platform/turtlebot3/challenges/#autorace-rbiz-challenge")
* 2. Features
	+ [2. 1. Specifications](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications "https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications")
	+ [2. 2. Components](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#components "https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#components")
* 3. Quick Start Guide
	+ [3. 1. PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup")
	+ [3. 2. SBC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup")
	+ [3. 3. OpenCR Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup")
	+ [3. 4. Hardware Assembly](https://emanual.robotis.com/docs/en/platform/turtlebot3/hardware_setup/#hardware-assembly "https://emanual.robotis.com/docs/en/platform/turtlebot3/hardware_setup/#hardware-assembly")
	+ [3. 5. Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup "https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup")
	+ 3. 6. Basic Operation
		- [- Teleoperation](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#teleoperation "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#teleoperation")
		- [- Topic Monitor](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#topic-monitor "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#topic-monitor")
* 4. SLAM
	+ [4. 1. Run SLAM Node](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node")
	+ [4. 2. Run Teleoperation Node](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-teleoperation-node "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-teleoperation-node")
	+ [4. 3. Tuning Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#tuning-guide "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#tuning-guide")
	+ [4. 4. Save Map](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#save-map "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#save-map")
	+ [4. 5. Map](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#map "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#map")
* 5. Navigation
	+ [5. 1. Run Navigation Nodes](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes "https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes")
	+ [5. 2. Estimate Initial Pose](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#estimate-initial-pose "https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#estimate-initial-pose")
	+ [5. 3. Set Navigation Goal](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#set-navigation-goal "https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#set-navigation-goal")
	+ [5. 4. Tuning Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#tuning-guide "https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#tuning-guide")
* 6. Simulation
	+ [6. 1. Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation "https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation")
	+ [6. 2. SLAM Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/")
	+ [6. 3. Navigation Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/")
	+ [6. 4. Fake Node Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/fakenode_simulation/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/fakenode_simulation/")
	+ [6. 5. Standalone Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/standalone_gazebo_simulation/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/standalone_gazebo_simulation/")
* 7. Manipulation
	+ [7. 1. TB3 & OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#turtlebot3-with-openmanipulator "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#turtlebot3-with-openmanipulator")
	+ [7. 2. Software Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#software-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#software-setup")
	+ [7. 3. Hardware Assembly](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#hardware-assembly "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#hardware-assembly")
	+ [7. 4. OpenCR Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#opencr-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#opencr-setup")
	+ [7. 5. Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#bringup "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#bringup")
	+ [7. 6. Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation")
	+ [7. 7. Operate the Actual OpenMANIPULATOR](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#operate-the-actual-openmanipulator "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#operate-the-actual-openmanipulator")
	+ [7. 8. SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#slam "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#slam")
	+ [7. 9. Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#navigation "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#navigation")
	+ 7. 10. Home Service Challenge
		- [- Getting Started](https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#getting-started "https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#getting-started")
		- [- Camera Calibration](https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#camera-calibration "https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#camera-calibration")
		- [- SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#slam "https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#slam")
		- [- Missions](https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#missions "https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#missions")
		- [- Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#simulation "https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/#simulation")
* 8. Autonomous Driving
	+ [8. 1. Getting Started](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#getting-started "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#getting-started")
	+ 8. 2. Camera Calibration
		- [- Camera Imaging Calibration](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#camera-imaging-calibration "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#camera-imaging-calibration")
		- [- Intrinsic Camera Calibration](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#intrinsic-camera-calibration "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#intrinsic-camera-calibration")
		- [- Extrinsic Camera Calibration](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#extrinsic-camera-calibration "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#extrinsic-camera-calibration")
		- [- Check Calibration Result](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#check-calibration-result "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#check-calibration-result")
	+ [8. 3. Lane Detection](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#lane-detection "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#lane-detection")
	+ [8. 4. Traffic Sign Detection](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#traffic-sign-detection "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#traffic-sign-detection")
	+ 8. 5. Missions
		- [- Traffic Lights](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#traffic-lights "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#traffic-lights")
		- [- Intersection](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#intersection "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#intersection")
		- [- Construction](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#construction "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#construction")
		- [- Parking](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#parking "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#parking")
		- [- Level Crossing](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#level-crossing "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#level-crossing")
		- [- Tunnel](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#tunnel "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#tunnel")
	+ [8. 6. TurtleBot3 AutoRace 2019](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving_autorace/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving_autorace/")
* 9. Machine Learning
	+ [9. 1. Software Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#software-setup "https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#software-setup")
	+ [9. 2. Set parameters](https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#set-parameters "https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#set-parameters")
	+ [9. 3. Run Machine Learning](https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#run-machine-learning "https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/#run-machine-learning")
* 10. Examples
	+ [10. 1. Interactive Markers](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#move-using-interactive-markers "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#move-using-interactive-markers")
	+ [10. 2. Obstacle Detection](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#obstacle-detection "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#obstacle-detection")
	+ [10. 3. Position Control](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#position-control "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#position-control")
	+ [10. 4. Point Operation](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#point-operation "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#point-operation")
	+ [10. 5. Patrol](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#patrol "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#patrol")
	+ [10. 6. TurtleBot Follower Demo](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#turtlebot-follower-demo "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#turtlebot-follower-demo")
	+ [10. 7. TurtleBot Panorama Demo](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#turtlebot-panorama-demo "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#turtlebot-panorama-demo")
	+ [10. 8. Automatic Parking](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#automatic-parking "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#automatic-parking")
	+ [10. 9. Automatic Parking Vision](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#automatic-parking-vision "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#automatic-parking-vision")
	+ [10. 10. Load Multiple TurtleBot3s](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#load-multiple-turtlebot3s "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/#load-multiple-turtlebot3s")
* 11. Friends(Locomotion)
	+ [11. 1. TurtleBot3 Friends: Car](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-car "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-car")
	+ [11. 2. TurtleBot3 Friends: OpenMANIPULATOR](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-openmanipulator "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-openmanipulator")
	+ [11. 3. TurtleBot3 Friends: Segway](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-segway "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-segway")
	+ [11. 4. TurtleBot3 Friends: Conveyor](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-conveyor "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-conveyor")
	+ [11. 5. TurtleBot3 Friends: Monster](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-monster "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-monster")
	+ [11. 6. TurtleBot3 Friends: Tank](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-tank "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-tank")
	+ [11. 7. TurtleBot3 Friends: Omni](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-omni "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-omni")
	+ [11. 8. TurtleBot3 Friends: Mecanum](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-mecanum "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-mecanum")
	+ [11. 9. TurtleBot3 Friends: Bike](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-bike "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-bike")
	+ [11. 10. TurtleBot3 Friends: Road Train](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-road-train "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-road-train")
	+ [11. 11. TurtleBot3 Friends: Real TurtleBot](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-real-turtlebot "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-real-turtlebot")
	+ [11. 12. TurtleBot3 Friends: Carrier](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-carrier "https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-carrier")
* 12. Learn
	+ [12. 1. AWS RoboMaker with TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#aws-robomaker-with-turtlebot3 "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#aws-robomaker-with-turtlebot3")
	+ [12. 2. Data Collection via Matlab](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#data-collection-via-matlab "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#data-collection-via-matlab")
	+ [12. 3. TurtleBot3 Blockly](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#turtlebot3-blockly "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#turtlebot3-blockly")
	+ [12. 4. TurtleBot3 Simulation on ROS Indigo](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#turtlebot3-simulation-on-ros-indigo "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#turtlebot3-simulation-on-ros-indigo")
	+ [12. 5. Youtube Course](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#youtube-course "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#youtube-course")
	+ [12. 6. Books](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#books "https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#books")
	+ 12. 7. Videos
		- [- Open Source Team](https://emanual.robotis.com/docs/en/platform/turtlebot3/videos/#videos-from-open-source-team "https://emanual.robotis.com/docs/en/platform/turtlebot3/videos/#videos-from-open-source-team")
		- [- ROBOTIS Channel](https://emanual.robotis.com/docs/en/platform/turtlebot3/videos/#videos-from-robotis-channel "https://emanual.robotis.com/docs/en/platform/turtlebot3/videos/#videos-from-robotis-channel")
		- [- Projects](https://emanual.robotis.com/docs/en/platform/turtlebot3/projects/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/projects/")
* 13. More Info
	+ 13. 1. Appendixes
		- [- DYNAMIXEL](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendixes/#dynamixel "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendixes/#dynamixel")
		- [- OpenCR1.0](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_opencr1_0/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_opencr1_0/")
		- [- LDS-01](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/")
		- [- LDS-02](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_02/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_02/")
		- [- RealSense™](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_realsense/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_realsense/")
		- [- Raspberry Pi Camera](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/")
	+ [13. 2. Compatible Devices](https://emanual.robotis.com/docs/en/platform/turtlebot3/compatible_devices/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/compatible_devices/")
	+ [13. 3. Additional Sensors](https://emanual.robotis.com/docs/en/platform/turtlebot3/additional_sensors/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/additional_sensors/")
	+ 13. 4. OpenSource and Licenses
		- [- OpenSource Software](https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#opensource-software "https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#opensource-software")
		- [- OpenSource Hardware](https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#opensource-hardware "https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#opensource-hardware")
		- [- Software License](https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#software-license "https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#software-license")
		- [- Hardware License](https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#hardware-license "https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#hardware-license")
		- [- Documents License](https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#documents-license "https://emanual.robotis.com/docs/en/platform/turtlebot3/opensource/#documents-license")
	+ 13. 5. Contact US
		- [- About Open Robotics](https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-open-robotics "https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-open-robotics")
		- [- About ROBOTIS](https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-robotis "https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-robotis")
		- [- About OST (Open Source Team)](https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-ost-open-source-team "https://emanual.robotis.com/docs/en/platform/turtlebot3/contact_us/#about-ost-open-source-team")
* [14. FAQ](https://emanual.robotis.com/docs/en/platform/turtlebot3/faq/#faq "https://emanual.robotis.com/docs/en/platform/turtlebot3/faq/#faq")

[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/basic_examples.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/basic_examples.md") 

Kinetic 
Melodic
Noetic
Dashing
Foxy
Humble
Windows

# [Examples](#examples "#examples")

**WARNING**: Make sure to run the [Bringup](/docs/en/platform/turtlebot3/bringup/#bringup "/docs/en/platform/turtlebot3/bringup/#bringup") instruction before performing this examples, and be careful when testing the robot on the table as the robot might fall.

**NOTE**:

* This instruction was tested on `Ubuntu 16.04` and `ROS Kinetic Kame`.
* This instruction is supposed to be running on the remote PC. Follow the instruction on your **Remote PC**.

The contents in e-Manual are subject to be updated without a prior notice. Therefore, some video may differ from the contents in e-Manual.

**NOTE**: This feature is available for ROS Kinetic and ROS2 Dashing.

**NOTE**: This feature is available for ROS Kinetic and ROS2 Dashing.

**NOTE**:

* This instruction was tested on `Ubuntu 18.04` and `ROS Dashing Diademata`.
* This instruction is supposed to be run on the remote PC. Follow the instruction on your **Remote PC**.

**CAUTION**: Place the robot on a level surface to ensure that it can not fall down from a table.

**WARNING**: Make sure to run the [Bringup](/docs/en/platform/turtlebot3/bringup/#bringup "/docs/en/platform/turtlebot3/bringup/#bringup") instruction before performing this examples.

**NOTE**: This feature is available for ROS Kinetic and ROS2 Dashing.

**NOTE**: This feature is available for ROS Kinetic and ROS2 Dashing.

**NOTE**: This feature is available for ROS Kinetic and ROS2 Dashing.

## [Move using Interactive Markers](#move-using-interactive-markers "#move-using-interactive-markers")

The TurtleBot3 can be moved by [Interactive Markers](http://wiki.ros.org/interactive_markers "http://wiki.ros.org/interactive_markers") on RViz. You can move the TurtleBot3 to rotate or linear using Interactive Markers.

**[Remote PC]** Open a new terminal and launch the remote file.

**TIP**: Before executing this command, you have to specify the model name of TurtleBot3. The `${TB3_MODEL}` is the name of the model you are using in `burger`, `waffle`, `waffle_pi`. If you want to permanently set the export settings, please refer to [Export TURTLEBOT3\_MODEL](/docs/en/platform/turtlebot3/export_turtlebot3_model "/docs/en/platform/turtlebot3/export_turtlebot3_model") page.

```
$ export TURTLEBOT3\_MODEL=${TB3\_MODEL}
$ roslaunch turtlebot3_bringup turtlebot3_remote.launch

```

**[Remote PC]** launch the interactive markers file.

```
$ roslaunch turtlebot3_example interactive_markers.launch

```

**[Remote PC]** Visualize the model in 3D with RViz.

```
$ rosrun rviz rviz -d `rospack find turtlebot3_example`/rviz/turtlebot3_interactive.rviz

```

**NOTE**: This feature is available for Kinetic.

## [Obstacle Detection](#obstacle-detection "#obstacle-detection")

The TurtleBot3 can be moved or stopped by LDS data. When the TurtleBot3 moves, it stops when it detects an obstacle ahead.

**[Remote PC]** Launch the obstacle file.

```
$ roslaunch turtlebot3_example turtlebot3_obstacle.launch

```

TurtleBot3 can moves with receiving a certain topic: **/cmd\_vel\_raw** from obstacle detection node. Using LDS data and detecting obstacles, the robot can stop moving.

**[Remote PC]**

1. Open a terminal on **Remote PC**.
2. Run the teleoperation keyboard.

```
  $ ros2 run turtlebot3_teleop teleop_keyboard /cmd_vel:=/cmd_vel_raw

```
3. Launch the obstacle detection.

```
  $ ros2 launch turtlebot3_example turtlebot3_obstacle_detection

```

## [Position Control](#position-control "#position-control")

**NOTE**: This feature is available for Dashing.

TurtleBot3 can be position controlled using this package.

**[Remote PC]**

1. Open a terminal on **Remote PC**.
2. Launch the pointop file.

```
  $ ros2 run turtlebot3_example turtlebot3_position_control

```
3. Type input values and press enter for `x`, `y` and `theta` respectively. TurtleBot3 will move to a pose (`x`, `y`, `theta`) from the current pose.

```
 TurtleBot3 Position Control
 ------------------------------------------------------
 From the current pose,
 x: goal position x (unit: m)
 y: goal position y (unit: m)
 theta: goal orientation (range: -180 ~ 180, unit: deg)
 ------------------------------------------------------

 Input x: 1  (input a value and press enter)
 Input y: 1  (input a value and press enter)
 Input theta: 10  (input a value and press enter)

```

## [Point Operation](#point-operation "#point-operation")

The TurtleBot3 can be moved by 2D `point (x, y)` and `z-angular`. For example, if you insert (0.5, 0.3, 60), TurtleBot3 moves to point (x = 0.5m, y = 0.3m) and then rotates 60 deg.

**[Remote PC]** launch the pointop file.

```
$ roslaunch turtlebot3_example turtlebot3_pointop_key.launch

```

TurtleBot3 can moves with receiving a certain topic: **/cmd\_vel\_raw** from obstacle detection node. Using LDS data and detecting obstacles, the robot can stop moving.

**[Remote PC]**

1. Open a terminal on **Remote PC**.
2. Run the teleoperation keyboard.

```
  $ ros2 run turtlebot3_teleop teleop_keyboard /cmd_vel:=/cmd_vel_raw

```
3. Launch the obstacle detection.

```
  $ ros2 launch turtlebot3_example turtlebot3_obstacle_detection

```

## [Patrol](#patrol "#patrol")

The TurtleBot3 can be moved by custom routes. There are three routes(rectangle, triangle and circle). This example uses action topic. Action client translates patrol data(mode, area, count) to action server. And then action server translates `cmd_vel` to TurtleBot3. Please refer to the above [tutorial video](https://youtu.be/Xg1pKFQY5p4 "https://youtu.be/Xg1pKFQY5p4") for more detailed usage.

**[Remote PC]** Launch the patrol server file.

```
$ rosrun turtlebot3_example turtlebot3_server

```

**[Remote PC]** Launch the patrol client file.

```
$ roslaunch turtlebot3_example turtlebot3_client.launch

```

This example uses action topic. The action client translates patrol data(radius) to action server. Then, the action server translates `cmd_vel` to TurtleBot3.

**[Remote PC]**

1. Open a terminal on **Remote PC**.
2. Launch the patrol server file.

```
  $ ros2 run turtlebot3_example turtlebot3_patrol_server

```
3. Launch the patrol client file.

```
  $ ros2 run turtlebot3_example turtlebot3_patrol_client

```
4. Type a input value and press enter. TurtleBot3 will draw a circle of the typed radius.

```
 TurtleBot3 Circle Patrol
 ------------------------------------------------------
 radius: circle radius (unit: m)
 ------------------------------------------------------

 Input radius: 1  (input a value and press enter)

```

## [TurtleBot Follower Demo](#turtlebot-follower-demo "#turtlebot-follower-demo")

In order to try examples in the below, you have to install the [turtlebot3\_applications](https://github.com/ROBOTIS-GIT/turtlebot3_applications "https://github.com/ROBOTIS-GIT/turtlebot3_applications") and [turtlebot3\_applications\_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs "https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs") packages.

**[Remote PC]** Go to `catkin workspace` directory (/home/(user\_name)/catkin\_ws/src) and clone the turtlebot3\_applications and turtlebot3\_applications\_msgs repository. Then run the `catkin_make` to build the new packages.

```
$ sudo apt-get install ros-kinetic-ar-track-alvar
$ sudo apt-get install ros-kinetic-ar-track-alvar-msgs
$ cd ~/catkin_ws/src
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs.git
$ cd ~/catkin_ws && catkin_make

```

**NOTE**:

* The follower demo was implemented only using a 360 Laser Distance Sensor LDS-01. A classification algorithm is used based on previous fitting with samples of person and obstacles positions to take actions. It follows someone in front of the robot within a 50 centimeter range and 140 degrees.
* Running the follower demo in an area with obstacles may not work well. Therefore, it is recommended to run the demo in an open area without obstacles.

1. **[Remote PC]** Install `scikit-learn`, `NumPy` and `ScyPy` packages.

```
$ sudo apt-get install python-pip
$ sudo pip install -U scikit-learn numpy scipy
$ sudo pip install --upgrade pip

```
2. **[Remote PC]** When installation is completed, run roscore on the remote pc.

```
$ roscore

```
3. **[TurtleBot]** Launch the [Bringup](/docs/en/platform/turtlebot3/bringup/#bringup "/docs/en/platform/turtlebot3/bringup/#bringup") package.

```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch

```
4. **[Remote PC]** Set the enviroment variable for the TurtleBot3 Burger and launch `turtlebot3_follow_filter`.

```
$ export TURTLEBOT3\_MODEL=burger
$ roslaunch turtlebot3_follow_filter turtlebot3_follow_filter.launch

```
5. **[Remote PC]** Launch `turtlebot3_follower`.

```
$ roslaunch turtlebot3_follower turtlebot3_follower.launch

```

**NOTE**: This feature is available for Kinetic.

## [TurtleBot Panorama Demo](#turtlebot-panorama-demo "#turtlebot-panorama-demo")

**NOTE**:

* The `turtlebot3_panorama` demo uses `pano_ros` for taking snapshots and stitching them together to create panoramic image.
* Panorama demo requires to install `raspicam_node` package. Instructions for installing this package can be found at [Gihub Link](https://github.com/UbiquityRobotics/raspicam_node "https://github.com/UbiquityRobotics/raspicam_node")
* Panorama demo requires to install OpenCV and cvbridge packages. Instructions for installing OpenCV can be found at [OpenCV Tutorial Link](http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html "http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html")

1. **[TurtleBot]** Launch the `turtlebot3_rpicamera` file

```
$ roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch

```
2. **[Remote PC]** Launch `panorama`.

```
$ roslaunch turtlebot3_panorama panorama.launch

```
3. **[Remote PC]** To start the panorama demo, enter the following command.

```
$ rosservice call turtlebot3_panorama/take_pano 0 360.0 30.0 0.3

```

Parameters that can be sent to the rosservice to get a panoramic image are:

	* Mode for taking the pictures.
	* 0 : snap&rotate (i.e. rotate, stop, snapshot, rotate, stop, snapshot, …)
	* 1 : continuous (i.e. keep rotating while taking snapshots)
	* 2 : stop taking pictures and create panoramic image
	 - Total angle of panoramic image, in degrees
	 - Angle interval (in degrees) when creating the panoramic image in snap&rotate mode, time interval (in seconds) otherwise
	 - Rotating velocity (in radians/s)
4. **[Remote PC]** To view the result image, enter the following command.

```
$ rqt_image_view image:=/turtlebot3_panorama/panorama

```

![](/assets/images/platform/turtlebot3/application/panorama_view.png)

**NOTE**: This feature is available for Kinetic.

## [Automatic Parking](#automatic-parking "#automatic-parking")

**NOTE**:

* The `turtlebot3_automatic_parking` demo was using a 360 laser Distance Sensor LDS-01 and a reflective tape. The LaserScan topic has intensity and distance data from LDS. The TurtleBot3 uses this to locate the reflective tape.
* The `turtlebot3_automatic_parking` demo requires `NumPy` package.

1. **[Remote PC]** Install `NumPy` package with below commands. If you already installed numpy, you can **skip** below commands.

```
$ sudo apt-get install python-pip
$ sudo pip install -U numpy
$ sudo pip install --upgrade pip

```
2. **[Remote PC]** Run roscore.

```
$ roscore

```
3. **[TurtleBot]** Bring up basic packages to start TurtleBot3 applications.

```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch

```
4. **[Remote PC]** If you use TurtleBot3 Burger, set the model of TurtleBot3 like command below.

```
$ export TURTLEBOT3\_MODEL=burger

```

**NOTE**: Specify `${TB3_MODEL}`: `burger`, `waffle`, `waffle_pi` before excuting the command. Set the permanent export setting by following [Export TURTLEBOT3\_MODEL](/docs/en/platform/turtlebot3/export_turtlebot3_model "/docs/en/platform/turtlebot3/export_turtlebot3_model") instruction.
5. **[Remote PC]** Run RViz.

```
$ roslaunch turtlebot3_bringup turtlebot3_remote.launch
$ rosrun rviz rviz -d `rospack find turtlebot3_automatic_parking`/rviz/turtlebot3_automatic_parking.rviz

```
6. **[Remote PC]** Launch the automatic parking file.

```
$ roslaunch turtlebot3_automatic_parking turtlebot3_automatic_parking.launch

```

	* You can select LaserScan topic in RViz.
	* `/scan`![](/assets/images/platform/turtlebot3/application/scan.png)

	* `/scan_spot`![](/assets/images/platform/turtlebot3/application/scan_spot.png)

In order to try examples in the below, you have to install the [turtlebot3\_applications](https://github.com/ROBOTIS-GIT/turtlebot3_applications "https://github.com/ROBOTIS-GIT/turtlebot3_applications") package.

Go to `robotis workspace` directory (/home/(user\_name)/robotis\_ws/src) and clone the turtlebot3\_applications. Then run the `colcon build` to build the new packages.

1. Open a terminal on **Remote PC**.
2. Run the following commands.

```
$ cd ~/robotis_ws/src
$ git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_applications.git
$ cd ~/robotis_ws && colcon build --symlink-install

```

**NOTE**:

* The `turtlebot3_automatic_parking` demo uses a 360 laser Distance Sensor LDS-01 and a reflective tape. The LaserScan topic has intensity and distance data from LDS. The TurtleBot3 uses this to locate the reflective tape.
* The `turtlebot3_automatic_parking` demo requires `NumPy` package.

**[Remote PC]**

1. Open a terminal on **Remote PC**.
2. Install `NumPy` package using the following commands. Skip this step if `NumPy` package is already installed.

```
$ sudo apt-get install python-pip3
$ sudo pip3 install -U numpy
$ sudo pip3 install --upgrade pip

```

**[TurtleBot]**

1. Open a terminal on **TurtleBot3**.
2. Bring up basic packages to start TurtleBot3 applications using the following command.

```
$ ros2 launch turtlebot3_bringup robot.launch.py

```

**[Remote PC]**

1. Open a terminal on **Remote PC**.
2. Launch the automatic parking file using the following commands.

```
$ export TURTLEBOT3\_MODEL=${TB3\_MODEL}
$ ros2 launch turtlebot3_automatic_parking turtlebot3_automatic_parking.launch.py

```

**NOTE**: Specify `${TB3_MODEL}`: `burger`, `waffle`, `waffle_pi` before excuting the command. Set the permanent export setting by following [Export TURTLEBOT3\_MODEL](/docs/en/platform/turtlebot3/export_turtlebot3_model "/docs/en/platform/turtlebot3/export_turtlebot3_model") instruction.

## [Automatic Parking Vision](#automatic-parking-vision "#automatic-parking-vision")

**NOTE**:

* The `turtlebot3_automatic_parking_vision` uses raspberry pi camera and so the robot which is a default platform used for this demo is TurtleBot3 Waffle Pi. Since it parks from finding out AR marker on some wall, printed AR marker should be prepared. Whole process uses the image get from the camera, so if the process is not well being done, configure the parameters, such as brightness, contrast, etc.
* The `turtlebot3_automatic_parking_vision` uses rectified image based on `image_proc` nodes. To get rectified image, the robot should get optic calibration data for raspberry pi camera. (Every downloaded turtlebot3 packages already have the camera calibration data as raspberry pi camera v2 default.)
* The `turtlebot3_automatic_parking_vision` package requires `ar_track_alvar` package.

1. **[Remote PC]** Run roscore.

```
$ roscore

```
2. **[TurtleBot]** Bring up basic packages to start TurtleBot3 applications.

```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch

```
3. **[TurtleBot]** Start the raspberry pi camera nodes.

```
$ roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch

```
4. **[Remote PC]** Raspberry pi package will publish compressed type image for fast communication. However, what will be needed in image rectification in `image_proc` node is raw type image. Hence, compressed image should be transform to raw image.

```
$ rosrun image_transport republish compressed in:=raspicam_node/image raw out:=raspicam_node/image

```
5. **[Remote PC]** Then, the image rectification should be carried out.

```
$ ROS\_NAMESPACE=raspicam_node rosrun image_proc image_proc image_raw:=image \_approximate\_s=true _queue_size:=20

```
6. **[Remote PC]** Now should start the AR marker detection. Before running related launch file, the model of what will be used by this example code should be exported. After running the launch file, RViz will be automatically run under preset environments.

```
$ export TURTLEBOT3\_MODEL=waffle_pi
$ roslaunch turtlebot3_automatic_parking_vision turtlebot3_automatic_parking_vision.launch

```

> 
> TurtleBot3 Automatic Parking Vision
> 
> 
>

**NOTE**: This feature is available for Kinetic.

## [Load Multiple TurtleBot3s](#load-multiple-turtlebot3s "#load-multiple-turtlebot3s")

**NOTE**: This application must be set firmware version `1.2.1` or higher.

1. **[Remote PC]** Run roscore.

```
$ roscore

```
2. Bringup multiple turtlebot3s with different namespace. We recommend the namespace includes common words such as `tb3_0`, `tb3_1` or `my_robot_0`, `my_robot_1`

	* **[TurtleBot(tb3\_0)]** Bring up basic packages with `ROS NAMESPACE` for nodes, `multi_robot_name` for tf prefix and `set_lidar_frame_id` for lidar frame id. This parameters must be the same.

	```
	$ ROS\_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3\_0" set_lidar_frame_id:="tb3\_0/base\_scan"

	```
	* **[TurtleBot(tb3\_1)]** Bring up basic packages with `ROS NAMESPACE` for nodes, `multi_robot_name` for tf prefix and `set_lidar_frame_id` for lidar frame id. This parameters must be the same but different other robots.

	```
	$ ROS\_NAMESPACE=tb3_1 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3\_1" set_lidar_frame_id:="tb3\_1/base\_scan"

	```
3. Then the terminal you launched `tb3_0` will represents below messages. You can watch TF messages have prefix `tb3_0`

```
SUMMARY
========

PARAMETERS
 \* /rosdistro: kinetic
 \* /rosversion: 1.12.13
 \* /tb3_0/turtlebot3_core/baud: 115200
 \* /tb3_0/turtlebot3_core/port: /dev/ttyACM0
 \* /tb3_0/turtlebot3_core/tf_prefix: tb3_0
 \* /tb3_0/turtlebot3_lds/frame_id: tb3_0/base_scan
 \* /tb3_0/turtlebot3_lds/port: /dev/ttyUSB0

NODES
  /tb3_0/
    turtlebot3_core (rosserial_python/serial_node.py)
    turtlebot3_diagnostics (turtlebot3_bringup/turtlebot3_diagnostics)
    turtlebot3_lds (hls_lfcd_lds_driver/hlds_laser_publisher)

ROS\_MASTER\_URI=http://192.168.1.2:11311

process[tb3_0/turtlebot3_core-1]: started with pid [1903]
process[tb3_0/turtlebot3_lds-2]: started with pid [1904]
process[tb3_0/turtlebot3_diagnostics-3]: started with pid [1905]
[INFO] [1531356275.722408]: ROS Serial Python Node
[INFO] [1531356275.796070]: Connecting to /dev/ttyACM0 at 115200 baud
[INFO] [1531356278.300310]: Note: publish buffer size is 1024 bytes
[INFO] [1531356278.303516]: Setup publisher on sensor_state [turtlebot3_msgs/SensorState]
[INFO] [1531356278.323360]: Setup publisher on version_info [turtlebot3_msgs/VersionInfo]
[INFO] [1531356278.392212]: Setup publisher on imu [sensor_msgs/Imu]
[INFO] [1531356278.414980]: Setup publisher on cmd_vel_rc100 [geometry_msgs/Twist]
[INFO] [1531356278.449703]: Setup publisher on odom [nav_msgs/Odometry]
[INFO] [1531356278.466352]: Setup publisher on joint_states [sensor_msgs/JointState]
[INFO] [1531356278.485605]: Setup publisher on battery_state [sensor_msgs/BatteryState]
[INFO] [1531356278.500973]: Setup publisher on magnetic_field [sensor_msgs/MagneticField]
[INFO] [1531356280.545840]: Setup publisher on /tf [tf/tfMessage]
[INFO] [1531356280.582609]: Note: subscribe buffer size is 1024 bytes
[INFO] [1531356280.584645]: Setup subscriber on cmd_vel [geometry_msgs/Twist]
[INFO] [1531356280.620330]: Setup subscriber on sound [turtlebot3_msgs/Sound]
[INFO] [1531356280.649508]: Setup subscriber on motor_power [std_msgs/Bool]
[INFO] [1531356280.688276]: Setup subscriber on reset [std_msgs/Empty]
[INFO] [1531356282.022709]: Setup TF on Odometry [tb3_0/odom]
[INFO] [1531356282.026863]: Setup TF on IMU [tb3_0/imu_link]
[INFO] [1531356282.030138]: Setup TF on MagneticField [tb3_0/mag_link]
[INFO] [1531356282.033628]: Setup TF on JointState [tb3_0/base_link]
[INFO] [1531356282.041117]: --------------------------
[INFO] [1531356282.044421]: Connected to OpenCR board!
[INFO] [1531356282.047700]: This core(v1.2.1) is compatible with TB3 Burger
[INFO] [1531356282.051355]: --------------------------
[INFO] [1531356282.054785]: Start Calibration of Gyro
[INFO] [1531356284.585490]: Calibration End

```
4. **[Remote PC]** Launch robot state publisher with same namespace.
	* **[TurtleBot(tb3\_0)]**

	```
	$ ROS\_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_remote.launch multi_robot_name:=tb3_0

	```
	* **[TurtleBot(tb3\_1)]**

	```
	$ ROS\_NAMESPACE=tb3_1 roslaunch turtlebot3_bringup turtlebot3_remote.launch multi_robot_name:=tb3_1

	```
5. Before start another application, check topics and TF tree to open rqt

```
$ rqt

```

![](/assets/images/platform/turtlebot3/application/multi_turtlebot_rqt.png)

To use this setup, each turtlebot3 makes map using SLAM and these maps are merged simutaneously by [multi\_map\_merge](http://wiki.ros.org/multirobot_map_merge "http://wiki.ros.org/multirobot_map_merge") packages. You can get more information about this to visit [Virtual SLAM by Multiple TurtleBot3s](/docs/en/platform/turtlebot3/simulation/#2-excute-slam "/docs/en/platform/turtlebot3/simulation/#2-excute-slam") sections

**NOTE**: This feature is available for Kinetic.

 Previous Page
Next Page 

$reference = $(".archive");
$(window).resize(function() {
 var newWidth = $reference.width() \* 0.90;
 if (newWidth > 640) { newWidth = 640; }

 // Resize all videos according to aspect ratio
 $("iframe").each(
 function(index, elem) {
 elem.setAttribute("width", (newWidth).toFixed());
 elem.setAttribute("height", (newWidth\*0.56).toFixed());
 }
 );
// Kick off one resize to fix all videos on page load
}).resize();

© 2023 ROBOTIS. Powered by [Jekyll](http://jekyllrb.com "http://jekyllrb.com") & [Minimal Mistakes](https://mademistakes.com/work/minimal-mistakes-jekyll-theme/ "https://mademistakes.com/work/minimal-mistakes-jekyll-theme/").

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-83196341-13']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

 window.dataLayer = window.dataLayer || [];
 function gtag(){dataLayer.push(arguments);}
 gtag('js', new Date());

 gtag('config', 'G-155KHSC07K');

