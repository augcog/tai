

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

* [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/home_service_challenge/")
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

[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/manipulation/home_service_challenge.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/manipulation/home_service_challenge.md") 

Kinetic 
Noetic
Dashing
Foxy
Humble
Windows

## [TurtleBot3 Home Service Challenge](#turtlebot3-home-service-challenge "#turtlebot3-home-service-challenge")

**NOTE**:

* This instructions were tested on `Ubuntu 16.04` and `ROS Kinetic Kame`.
* For more informationn, see [OpenMANIPULATOR e-Manual](/docs/en/platform/openmanipulator/ "/docs/en/platform/openmanipulator/") and [[ROS 1] Turtlebot3 Manipulation](/docs/en/platform/turtlebot3/manipulation "/docs/en/platform/turtlebot3/manipulation")

![](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/hsc_stadium_2.png)

> 
> Home Service Challenge Stadium and Objects
> 
> 
> 

### [Getting Started](#getting-started "#getting-started")

**NOTE**: Be sure to complete the following instructions before installing Home Service Challenge packages in the pc.

* [TurtleBot3 PC Set up](/docs/en/platform/turtlebot3/quick-start/#pc-setup "/docs/en/platform/turtlebot3/quick-start/#pc-setup")
* [TurtleBot3 SBC Set up](/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup "/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup")
* [OpenMANIPULATOR-X](/docs/en/platform/openmanipulator_x/quick_start_guide/#install-ros-packages "/docs/en/platform/openmanipulator_x/quick_start_guide/#install-ros-packages") packages

#### [Prerequisites](#Prerequisites "#Prerequisites")

`TurtleBot3 Waffle Pi`

* It is the basic model to use Home Service Challenge packages for mobile manipulation.
* Provided source codes, Home Service Challenge Packages, are made based on TurtleBot3 Waffle Pi.

`OpenMANIPULATOR-X`

* OpenMANIPULATOR-X is an official robot platform to use mobile manipulation feature with Turtlebot3 in the Home Service Challenge.

`Remote PC`

* It communicates with an single board computer (SBC) on Turtlebot3.
* Laptop, desktop, or other devices with ROS 1.

#### [Remote PC Setup](#remote-pc-setup "#remote-pc-setup")

1. **[Remote PC]** Install Home Service Challenge packages.

```
$ cd ~/catkin_ws/src/
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_home_service_challenge.git
$ cd ~/catkin_ws && catkin_make

```
2. **[Remote PC]** Install additional dependent packages on Remote PC

```
$ sudo apt-get install ros-kinetic-ar-track-alvar ros-kinetic-ar-track-alvar-msgs

```
3. **[Remote PC]** Load the TurtleBot3 Waffle (or Waffle Pi) with OpenMANIPULATOR on RViz.

```
$ export TURTLEBOT3\_MODEL=${TB3\_MODEL}
$ roslaunch turtlebot3_manipulation_description turtlebot3_manipulation_view.launch use_gui:=true

```

**NOTE**: Specify `${TB3_MODEL}`: `waffle`, `waffle_pi` before excuting the command. Set the permanent export setting by following [Export TURTLEBOT3\_MODEL](/docs/en/platform/turtlebot3/export_turtlebot3_model "/docs/en/platform/turtlebot3/export_turtlebot3_model") instruction.

![turtlebot3_manipulation_view](/assets/images/platform/turtlebot3/manipulation/TurtleBot3_with_Open_Manipulator.png)

> 
> Rviz View
> 
> 
>
4. **[Remote PC]** In order to use your PC remotely, you need to make SSH keys. Create a script file to generate SSH key.

```
$ nano ~/tb3_ssh_keygen

```
5. **[Remote PC]** Copy and paste the followings to the script, and save it.

```
#!/bin/bash
argc=$#
args=("$@")
if [ 0 -eq $argc ]
then
 echo "need to argument that host ip for ssh connection"
  echo "Usage: $0 [ip address] ..."
  exit 1
fi
for((index = 0; index < $#; index++ ))
do
 ssh-keygen -R ${args[$index]}
  ssh-keyscan ${args[$index]} >> ~/.ssh/known_hosts
done

```
6. **[Remote PC]** Make the file executable.

```
$ chmod +x ~/tb3_ssh_keygen

```
7. **[Remote PC]** Execute the file with TurtleBot3’s IP address. Be sure **Remote PC** and **TurtleBot3** are connected **under the same WiFi**.

```
$ ~/tb3_ssh_keygen {IP_ADDRESS_OF_TURTLEBOT3}

```
8. Check `~/.ssh/known_hosts` file if SSH is successfully generated.

![known_hosts](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/known_hosts.png)

#### [SBC Setup](#sbc-setup "#sbc-setup")

1. **[Turtlebot3 SBC]** Using a machine tag, create `env.bash` file located in the **/home/pi**.

```
$ nano ~/env.bash

```
2. **[Turtlebot3 SBC]** Copy and paste the followings to the script, and save it.

```
#!/bin/bash
# check if other turtlebot3\_core is already running
is\_running=`ps ax | grep turtlebot3_core`
IFS=' ' read -ra is_runnings <<< "$is\_running"
process\_name=${is\_runnings[4]}
if [ ${process\_name} == "python" ]
then
 echo "other turtlebot3\_core is already running."
  exit 1
fi
#### ROS ####
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
#### ROS Network ####
ip\_address=`hostname -I`
ip\_address\_trim=${ip\_address%% \* }
ip\_address\_no\_space="$(echo -e "${ip\_address\_trim}" | tr -d '[:space:]')"
export ROS\_HOSTNAME=${ip\_address\_no\_space}
##### Set TURTLEBOT3 Model ####
export TURTLEBOT3\_MODEL=waffle_pi
exec "$@"

```
3. Make the file executable.

```
$ chmod +x ~/env.bash

```

#### [OpenCR Setup](#opencr-setup "#opencr-setup")

* See the [OpenCR Setup](/docs/en/platform/turtlebot3/manipulation/#opencr-setup "/docs/en/platform/turtlebot3/manipulation/#opencr-setup")

#### [Hardware Setup](#hardware-setup "#hardware-setup")

* See the [Hardware Setup](/docs/en/platform/turtlebot3/manipulation/#hardware-setup "/docs/en/platform/turtlebot3/manipulation/#hardware-setup")

### [Camera Calibration](#camera-calibration "#camera-calibration")

Calibrate a camera to decrease or minimize distortion. Follow the camera calibration process step by step.

#### [Camera Setting](#camera-setting "#camera-setting")

1. **[Remote PC]** Use the following command to run a camera remotely. Make sure to type IP address where TurtleBot3 and remote pc are connected together.  

 Be sure **Remote PC** and **TurtleBot3** are connected **under the same IP**.

```
$ roslaunch turtlebot3_home_service_challenge_tools turtlebot3_pi_cam_remote.launch address:=TURTLEBOT_IP

```
2. **[Remote PC]** Execute rqt\_image\_view.

```
$ rqt_image_view

```
3. Select **/raspicam\_node/image/compressed** (or **/raspicam\_node/image/**) topic on the check box.

![camera view](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/camera_setting_01.png)
4. **[Remote PC]** Excute rqt\_reconfigure.

```
$ rosrun rqt_reconfigure rqt_reconfigure

```
5. Click **raspicam\_node**, and modify parameter value to see clear images from the camera.

![camera setting](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/camera_setting_02.png)
6. Open **camera.yaml** file located in **turtlebot3\_home\_service\_challenge\_tools/config/camera\_calibration**.
7. Write modified values to the file.

```
ISO: 400
awb_mode: auto
brightness: 50
contrast: 0
exposureCompensation: 0
exposure_mode: auto
hFlip: false
saturation: 0
sharpness: 0
shutterSpeed: 0
vFlip: false
videoStabilisation: false
zoom: 1.0

```

#### [Intrinsic Camera Calibration](#intrinsic-camera-calibration "#intrinsic-camera-calibration")

Print a checkerboard on A4 size paper. The checkerboard is used for Intrinsic Camera Calibration.

* Download a checkerboard from [ROS Wiki](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check-108.pdf "http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check-108.pdf")
* For detailed information on the camera calibration, see [camera\_calibration](http://wiki.ros.org/camera_calibration "http://wiki.ros.org/camera_calibration") from ROS Wiki.

![checkerboard](/assets/images/platform/turtlebot3/autonomous_driving/autorace_checkerboard.png)

> 
> Checkerboard
> 
> 
>

1. **[Remote PC]** Use the following command to run a camera remotely. Make sure to include IP address when using the command.

```
$ roslaunch turtlebot3_home_service_challenge_tools turtlebot3_pi_cam_remote.launch enable_raw:=true address:=TURTLEBOT_IP

```

**NOTE**: Be sure **Remote PC** and **TurtleBot3** are connected **under the same IP**.
2. **[Remote PC]** Run a intrinsic camera calibration.

```
$ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/raspicam_node/image camera:=/raspicam_node

```

**NOTE**: The size of squres may differ depending on the print paper size A3, A4 or others. In which case, **adjust the value (0.024) of –square option in the given command** according to the square size in the print paper.

![calibration display](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/camera_calibration_01.png)
3. Use the checkerboard to calibrate the camera, and click **CALIBRATE**.

**NOTE**: Move and tilt the checker board around to the left, right, top, bottom and various angles to get `X`,`Y`, `Size` and `Skew` calibration data. As the data is computed enough, `X`,`Y`, `Size` and `Skew` will have a green bar.

![calibraion](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/camera_calibration_02.png)
4. Click **Commit** to save the intrinsic calibration data to the default folder.(~/.ros/camera\_info)

![commit](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/camera_calibration_04.png)
5. Open `camerav2_1280x720.yaml` file located **~/.ros/camera\_info** from `Turtlebot SBC` and check the saved data, which has the form like the script below.

```
image_width: 1280
image_height: 720
camera_name: camerav2_1280x720
 camera_matrix:
 rows: 3
 cols: 3
 data: [1280.628466572832, 0, 658.384219880362, 0, 1277.989132765768, 360.8363923435625, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
 rows: 1
 cols: 5
 data: [0.2032697817127332, -0.4147569733010375, -0.001420915782245743, 0.003110652248245379, 0]
rectification_matrix:
 rows: 3
 cols: 3
 data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
 rows: 3
 cols: 4
 data: [1312.630981445312, 0, 661.4149459665205, 0, 0, 1312.107055664062, 360.0239481801327, 0, 0, 0, 1, 0]

```

### [SLAM](#slam "#slam")

SLAM stands for Simultaneous Localization and Mapping. Using this feature, TurtleBot3 can draw a map. More information on this, see [SLAM](/docs/en/platform/turtlebot3/slam/#ros-1-slam "/docs/en/platform/turtlebot3/slam/#ros-1-slam")

1. **[Remote PC]** Use the following command to run a camera remotely. Make sure to include IP address when using the command.

```
$ roslaunch turtlebot3_home_service_challenge_tools turtlebot3_robot_remote.launch address:=TURTLEBOT_IP

```

**NOTE**: Be sure **Remote PC** and **TurtleBot3** are connected **under the same IP**.
2. **[Remote PC]** Run slam node.

```
$ roslaunch turtlebot3_home_service_challenge_tools slam.launch

```
3. **[Remote PC]** Run keyboard teleoperation node.

```
$ roslaunch turtlebot3_home_service_challenge_tools mobile_teleop_key.launch

```

After the node is run, the following controller will be displayed on a terminal window.

```
 Control Your TurtleBot3!
 ---------------------------
 Moving around:
   w
 a s d
   x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit

```
4. Update a map with the keyboard controller.

![making_map](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/slam_02.png)
5. **[Remote PC]** Save the map when it is fully updated.

```
$ ROS\_NAMESPACE=tb3_hsc rosrun map_server map_saver -f tb3_hsc

```

![map](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/map.png)

### [Missions](#missions "#missions")

#### [Run a Demo and Manager Pacakge](#run-a-demo-and-manager-package "#run-a-demo-and-manager-package")

1. **[Remote PC]** Run the demo pacakge. Make sure to include IP address when using the command. This package contains various features: Remote Bringup with a machine tag, Moveit, Navigation, Camera sensing

```
$ roslaunch turtlebot3_home_service_challenge_tools turtlebot3_home_service_challenge_demo_remote.launch address:=TURTLEBOT_IP

```

**NOTE**: Be sure **Remote PC** and **TurtleBot3** are connected **under the same IP**.

![demo_01](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/home_service_challenge_demo_01.png)
2. **[Remote PC]** Run the manager package used to carry out Home Service Challenge’s mission.

```
$ roslaunch turtlebot3_home_service_challenge_manager manager.launch

```

#### [Commands](#commands "#commands")

**[Remote PC]** Use the following commands during Home Service Challenge.

* **Ready** : TurtleBot3 will prepare to start a mission.

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String ready_mission

```
* **Start** : TurtleBot3 will start a mission.

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String start_mission

```
* **Stop** : TurtleBot3 will stop running a mission.

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String stop_mission

```
* **Restart** : TurtleBot3 will restart a mission by a given scenario.

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String restart_mission:SCENARIO_NAME

```

**NOTE**: When using this command, be sure to include one of the senario name from a `scenario.yaml` file. For detailed information on the scenario, see [Configuration](#configuration "#configuration") description below at this section.

#### [Operation Test](#operation-test "#operation-test")

**[Remote PC]** Publish the following topics to test a navigation or manipulation feature.

* Navigation

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String nav_start
$ rostopic pub -1 /tb3_hsc/command std_msgs/String nav_ar_marker_0
$ rostopic pub -1 /tb3_hsc/command std_msgs/String nav_ar_marker_1
$ rostopic pub -1 /tb3_hsc/command std_msgs/String nav_ar_marker_2

```
* Manipulation

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String arm_home
$ rostopic pub -1 /tb3_hsc/command std_msgs/String arm_joint
$ rostopic pub -1 /tb3_hsc/command std_msgs/String arm_task
$ rostopic pub -1 /tb3_hsc/command std_msgs/String open_gripper
$ rostopic pub -1 /tb3_hsc/command std_msgs/String close_gripper

```

#### [Configuration](#configuration "#configuration")

**[Remote PC]** Modify data in configuration files according to the given environment.

* `scenario.yaml` : This file contains a scenario’s data.
	+ File Path : **/turtlebot3\_home\_service\_challenge\_manager/script/scenario.yaml**
	+ Script

	```
	SCENARIO\_NAME: # start - scenario - finish
	  task: TASK\_NAME
	  args: [0, 1, 2]
	  timeout: 10 #sec, 0 : no time out
	  next\_scenario: find\_object
	  scenario\_on\_failure: standby
	  retry\_times: 0 #times, 0 : no retry

	```
* `room.yaml` : This file contains data of the Home Service Challenge’s stadium.
	+ File Path : **/turtlebot3\_home\_service\_challenge\_manager/config/room.yaml**
	+ Script

	```
	room\_1:
	  name: toilet
	  object:
	    marker: ar\_marker\_0
	    position: [0.25, 0, 0.15]
	  target:
	    marker: ar\_marker\_4
	    position: [0.25, 0, 0.15]
	  x: [1.5, 0.6]
	  y: [1.5, 0.2]

	```
* `config.yaml` : This configuration file contains manager package’s data.
	+ File Path : **/turtlebot3\_home\_service\_challenge\_manager/config/config.yaml**

#### [Details about the Home Service Mission](#details-about-the-home-service-mission "#details-about-the-home-service-mission")

The goal of the Home Service Challenge is to move four different objects from a living room to a specific place with given rules, and to return to the starting point.

Using the demo package, the process of moving objects in Home Service Challenge is as follows.

1. Navigating to a target in the living room.
	* Find a target, and reach it using a Navigation package.![demo_02](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/home_service_challenge_demo_02.png)
2. Approaching the target.
	* For the approach to the target with precise, TurtleBot3 wheels are directly controlled by computing target’s location from AR marker. (Used Topic : `/tb3_hsc/cmd_vel`). To produce a reliable performance, Closed-loop and control system can be used for the specified number of times.![demo_03](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/home_service_challenge_demo_03.png)
3. Picking the target with OpenMANIPULATOR-X’s gripper.
	* Pick the target object using the moveit package (Joint space control, Task space control and Gripper control)![manipulation_diagram](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/manipulation_diagram.png)

> 
> MoveIt Diagram
> 
> 
> 

![demo_04](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/home_service_challenge_demo_04.png)

> 
> Picking a Target Using MoveIt Package.
> 
> 
>
4. Leaving for the next room to place the object (Used Topic : `/tb3_hsc/cmd_vel`)
	* When moving back from the target, the wheels are directly controlled by the manager program using `/tb3_hsc/cmd_vel` topic.
5. Navigating to the place where the object will be placed.
	* Find a next target, and reach it using a Navigation package.![demo_05](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/home_service_challenge_demo_05.png)
6. Approaching the target.
7. Placing the object using the gripper.
8. Returning to the starting point using the navigation package.

![demo_06](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/home_service_challenge_demo_06.png)

### [Simulation](#simulation "#simulation")

Simulate TurtleBot3 with OpenMANIPULATOR-X in Gazebo.

1. **[Remote PC]** Run Gazebo.

```
$ roslaunch turtlebot3_home_service_challenge_simulation competition.launch

```

![gazebo](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/simulation_00.png)
2. **[Remote PC]** Run a simulation demo for Gazebo.

```
$ roslaunch turtlebot3_home_service_challenge_tools turtlebot3_home_service_challenge_demo_simulation.launch

```

![simulation_rviz](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/simulation_rviz_01.png)
3. **[Remote PC]** Run Home Service Manager.

```
$ roslaunch turtlebot3_home_service_challenge_manager manager.launch

```
4. Use the Home Service Challenge commands, See [Commands](#commands "#commands")

**NOTE**:

* This instructions were tested on `Ubuntu 20.04` and `ROS1 Noetic Ninjemys`.
* For more informationn, see [OpenMANIPULATOR e-Manual](/docs/en/platform/openmanipulator/ "/docs/en/platform/openmanipulator/") and [[ROS 1] Turtlebot3 Manipulation](/docs/en/platform/turtlebot3/manipulation "/docs/en/platform/turtlebot3/manipulation")
* Home Service Challenge noetic package is mainly tested under the **Gazebo simulation**.
* The actual robot will also be tested and updated.

![](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/hsc_stadium_2.png)

> 
> Home Service Challenge Stadium and Objects
> 
> 
> 

### [Getting Started](#getting-started "#getting-started")

**NOTE**: Be sure to complete the following instructions before installing Home Service Challenge packages in the pc.

* [TurtleBot3 PC Set up](/docs/en/platform/turtlebot3/quick-start/#pc-setup "/docs/en/platform/turtlebot3/quick-start/#pc-setup")
* [TurtleBot3 SBC Set up](/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup "/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup")
* [OpenMANIPULATOR-X](/docs/en/platform/openmanipulator_x/quick_start_guide/#install-ros-packages "/docs/en/platform/openmanipulator_x/quick_start_guide/#install-ros-packages") packages

#### [Prerequisites](#Prerequisites "#Prerequisites")

`Remote PC`

* ROS 1 Noetic installed Laptop or desktop PC.
* This instruction is based on Gazebo simulation.

#### [Remote PC Setup](#remote-pc-setup "#remote-pc-setup")

1. **[Remote PC]** Install Home Service Challenge packages.

```
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_home_service_challenge.git
$ git clone -b noetic-devel https://github.com/machinekoder/ar_track_alvar
$ cd ~/catkin_ws && catkin_make

```
2. **[Remote PC]** Load the TurtleBot3 Waffle (or Waffle Pi) with OpenMANIPULATOR on RViz.

```
$ export TURTLEBOT3\_MODEL=${TB3\_MODEL}
$ roslaunch turtlebot3_manipulation_description turtlebot3_manipulation_view.launch use_gui:=true

```

**NOTE**: Specify `${TB3_MODEL}`: `waffle`, `waffle_pi` before excuting the command. Set the permanent export setting by following [Export TURTLEBOT3\_MODEL](/docs/en/platform/turtlebot3/export_turtlebot3_model "/docs/en/platform/turtlebot3/export_turtlebot3_model") instruction.

![turtlebot3_manipulation_view](/assets/images/platform/turtlebot3/home_service_challenge/noetic/TurtleBot3_with_Open_Manipulator.png)

> 
> Rviz View. Specify ${TB3\_MODEL} : waffle\_pi
> 
> 
>

### [Camera Calibration](#camera-calibration "#camera-calibration")

**NOTE**: This chapter is for actual robots only. Gazebo does not proceed.

### [SLAM](#slam "#slam")

**NOTE**: This chapter is for actual robots only. Gazebo does not proceed.

### [Missions](#missions "#missions")

#### [Run a Demo and Manager Pacakge](#run-a-demo-and-manager-package "#run-a-demo-and-manager-package")

1. **[Remote PC]** Run the Gazebo Simulation.

```
$ roslaunch turtlebot3_home_service_challenge_simulation competition.launch

```
2. **[Remote PC]** Run a simulation demo for Gazebo.

```
$ roslaunch turtlebot3_home_service_challenge_tools turtlebot3_home_service_challenge_demo_simulation.launch

```

![simulation_rviz](/assets/images/platform/turtlebot3/home_service_challenge/noetic/simulation_rviz.png)
3. **[Remote PC]** Run the manager package used to carry out Home Service Challenge’s mission.

```
$ roslaunch turtlebot3_home_service_challenge_manager manager.launch

```

#### [Commands](#commands "#commands")

**[Remote PC]** Use the following commands during Home Service Challenge.

* **Ready** : TurtleBot3 will prepare to start a mission.

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String ready_mission

```
* **Start** : TurtleBot3 will start a mission.

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String start_mission

```
* **Stop** : TurtleBot3 will stop running a mission.

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String stop_mission

```
* **Restart** : TurtleBot3 will restart a mission by a given scenario.

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String restart_mission:SCENARIO_NAME

```

**NOTE**: When using this command, be sure to include one of the senario name from a `scenario.yaml` file. For detailed information on the scenario, see [Configuration](#configuration "#configuration") description below at this section.

#### [Operation Test](#operation-test "#operation-test")

**[Remote PC]** Publish the following topics to test a navigation or manipulation feature.

* Navigation

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String nav_start

```

![](/assets/images/platform/turtlebot3/home_service_challenge/noetic/mission_start.gif)

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String nav_ar_marker_0

```

![](/assets/images/platform/turtlebot3/home_service_challenge/noetic/mission_0.gif)

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String nav_ar_marker_1

```

![](/assets/images/platform/turtlebot3/home_service_challenge/noetic/mission_1.gif)

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String nav_ar_marker_2

```

![](/assets/images/platform/turtlebot3/home_service_challenge/noetic/mission_2.gif)

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String nav_ar_marker_3

```

![](/assets/images/platform/turtlebot3/home_service_challenge/noetic/mission_3.gif)
* Manipulation

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String arm_home

```

![](/assets/images/platform/turtlebot3/home_service_challenge/noetic/hsc_arm_home.gif)

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String arm_joint

```

![](/assets/images/platform/turtlebot3/home_service_challenge/noetic/hsc_arm_joint.gif)

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String arm_task

```

![](/assets/images/platform/turtlebot3/home_service_challenge/noetic/hsc_arm_task.gif)

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String open_gripper

```

![](/assets/images/platform/turtlebot3/home_service_challenge/noetic/hsc_open_gripper.gif)

```
$ rostopic pub -1 /tb3_hsc/command std_msgs/String close_gripper

```

![](/assets/images/platform/turtlebot3/home_service_challenge/noetic/hsc_close_gripper.gif)

#### [Configuration](#configuration "#configuration")

**[Remote PC]** Modify data in configuration files according to the given environment.

* `scenario.yaml` : This file contains a scenario’s data.
	+ File Path : **/turtlebot3\_home\_service\_challenge\_manager/script/scenario.yaml**
	+ Script

	```
	SCENARIO\_NAME: # start - scenario - finish
	  task: TASK\_NAME
	  args: [0, 1, 2]
	  timeout: 10 #sec, 0 : no time out
	  next\_scenario: find\_object
	  scenario\_on\_failure: standby
	  retry\_times: 0 #times, 0 : no retry

	```
* `room.yaml` : This file contains data of the Home Service Challenge’s stadium.
	+ File Path : **/turtlebot3\_home\_service\_challenge\_manager/config/room.yaml**
	+ Script

	```
	room\_1:
	  name: toilet
	  object:
	    marker: ar\_marker\_0
	    position: [0.25, 0, 0.15]
	  target:
	    marker: ar\_marker\_4
	    position: [0.25, 0, 0.15]
	  x: [1.5, 0.6]
	  y: [1.5, 0.2]

	```
* `config.yaml` : This configuration file contains manager package’s data.
	+ File Path : **/turtlebot3\_home\_service\_challenge\_manager/config/config.yaml**

#### [Details about the Home Service Mission](#details-about-the-home-service-mission "#details-about-the-home-service-mission")

The goal of the Home Service Challenge is to move four different objects from a living room to a specific place with given rules, and to return to the starting point.

Using the demo package, the process of moving objects in Home Service Challenge is as follows.

1. Navigating to a target in the living room.
	* Find a target, and reach it using a Navigation package.![demo_1](/assets/images/platform/turtlebot3/home_service_challenge/noetic/demo_1.PNG)
2. Approaching the target.
	* For the approach to the target with precise, TurtleBot3 wheels are directly controlled by computing target’s location from AR marker. (Used Topic : `/tb3_hsc/cmd_vel`). To produce a reliable performance, Closed-loop and control system can be used for the specified number of times.![demo_2](/assets/images/platform/turtlebot3/home_service_challenge/noetic/demo_2.PNG)
3. Picking the target with OpenMANIPULATOR-X’s gripper.
	* Pick the target object using the moveit package (Joint space control, Task space control and Gripper control)![manipulation_diagram](/assets/images/platform/turtlebot3/home_service_challenge/kinetic/manipulation_diagram.png)

> 
> MoveIt Diagram
> 
> 
>
4. Leaving for the next room to place the object (Used Topic : `/tb3_hsc/cmd_vel`)
	* When moving back from the target, the wheels are directly controlled by the manager program using `/tb3_hsc/cmd_vel` topic.
5. Navigating to the place where the object will be placed.
	* Find a next target, and reach it using a Navigation package.![demo_3](/assets/images/platform/turtlebot3/home_service_challenge/noetic/demo_3.PNG)
6. Approaching the target.
7. Placing the object using the gripper.
8. Returning to the starting point using the navigation package.

![demo_4](/assets/images/platform/turtlebot3/home_service_challenge/noetic/demo_4.PNG)

### [Simulation](#simulation "#simulation")

Simulate TurtleBot3 with OpenMANIPULATOR-X in Gazebo.

1. **[Remote PC]** Run Gazebo.

```
$ roslaunch turtlebot3_home_service_challenge_simulation competition.launch

```

![gazebo](/assets/images/platform/turtlebot3/home_service_challenge/noetic/simulation_gazebo.png)
2. **[Remote PC]** Run a simulation demo for Gazebo.

```
$ roslaunch turtlebot3_home_service_challenge_tools turtlebot3_home_service_challenge_demo_simulation.launch

```

![simulation_rviz](/assets/images/platform/turtlebot3/home_service_challenge/noetic/simulation_rviz.png)
3. **[Remote PC]** Run Home Service Manager.

```
$ roslaunch turtlebot3_home_service_challenge_manager manager.launch

```
4. Use the Home Service Challenge commands, See [Commands](#commands "#commands")

**NOTE**: This feature is available for Kinetic and Noetic.

**NOTE**: This feature is available for Kinetic and Noetic.

**NOTE**: This feature is available for Kinetic and Noetic.

**NOTE**: This feature is available for Kinetic and Noetic.

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

