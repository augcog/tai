

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

* [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/ "https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/")
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

[Edit on GitHub](https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/quick_start/sbc_setup.md "https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/platform/turtlebot3/quick_start/sbc_setup.md") 

Kinetic 
Melodic
Noetic
Dashing
Foxy
Humble
Windows

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* `Raspberry Pi 4`, `Jetson Nano` do not support ROS Kinetic.
* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.

If you are using **Intel Joule**, please refer to [Intel Joule Setup](/docs/en/popup/turtlebot3/joule_setup "/docs/en/popup/turtlebot3/joule_setup") instruction.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

### [Download TurtleBot3 SBC Image](#download-turtlebot3-sbc-image "#download-turtlebot3-sbc-image")

Download the correct image file for your hardware and ROS version.  

Kinetic version images use Raspberry Pi OS(Raspbian OS).

[![](/assets/images/icon_download.png)](http://www.robotis.com/service/download.php?no=1738 "http://www.robotis.com/service/download.php?no=1738")

**SHA256** : eb8173f3727db08087990b2c4e2bb211e70bd54644644834771fc8b971856b97

The recovery image files can be modified without a prior notice.

`Raspberry Pi 4` does not support Ubuntu 16.04 nor Debian Jessie, therefore, ROS Kinetic is not supported.

### [Unzip the downloaded image file](#unzip-the-downloaded-image-file "#unzip-the-downloaded-image-file")

Extract the `.img` file and save it in the local disk.

### Burn the image file

You can use various image burning tools.  

For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used.  

Choose your preferred tool to burn the image to microSD.

#### Raspberry Pi Imager

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png)](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)

1. Click `CHOOSE OS`.
2. Click `Use custom` and select the extracted `.img` file from local disk.
3. Click `CHOOSE STORAGE` and select the microSD.
4. Click `WRITE` to start burning the image.

#### Disks Utility

`Disks` utility is included in recent Ubuntu Desktop. Search for “Disks” and launch the app.

![](/assets/images/platform/turtlebot3/setup/disks.gif)

1. Select the microSD card in the left panel.
2. Select `Restore Disk Image` option.
3. Open the `.img` file from local disk.
4. Click `Start Restoring...` > `Restore` button.

![](/assets/images/icon_unfold.png)

In case you use Intel Joule 570x, please follow the instructions below.  

Intel Joule is discontinued in 2017, and additional support is unavailable.

1. Download Ubuntu 16.04 image for Intel® Joule™
	* [Download Ubuntu 16.04 for Intel® Joule™](http://people.canonical.com/~platform/snappy/tuchuck/desktop-final/tuchuck-xenial-desktop-iso-20170317-0.iso "http://people.canonical.com/~platform/snappy/tuchuck/desktop-final/tuchuck-xenial-desktop-iso-20170317-0.iso")
2. Create a bootable USB with the downloaded image.
3. Install Ubuntu from the USB

### Boot Up the Raspberry Pi

1. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.
2. Connect input devices to the USB port of Raspberry Pi
3. Insert the microSD card.
4. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.

### Configure the Raspberry Pi

**NOTE** : If you encounter apt failures about the ROS GPG key (due to the existing GPG expiration), you may need to update GPG key. Please see [ROS GPG Key Expiration Incident](#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669 "#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669"), and proceed to the given solution.

1. After Raspbian OS is up and running, connect to the WiFi network that is connected with the PC.
2. Find the assigned IP address for Raspberry Pi with the command below. Usually wireless IP address for Raspberry Pi can be found under the `wlan0` section.

```
$ ifconfig

```
3. From your PC, open the terminal and connect to the Raspberry Pi with its IP address.  

 The default password is set as **turtlebot**.

```
$ ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}

```
4. Once logged in to the Raspberry Pi, execute the commands below to sync time.

```
$ sudo apt-get install ntpdate
$ sudo ntpdate ntp.ubuntu.com

```
5. Load Raspberry Pi configuration interface.

```
$ sudo raspi-config

```
6. Select `Advanced Options` > `Expand Filesystem` and exit.
7. Network configuration for ROS

```
$ nano ~/.bashrc

```
8. Go to the end of file with `Ctrl`+`END` or `Alt`+`/`, then modify the IP adddresses of `ROS_MASTER_URI` and the `ROS_HOSTNAME`.

```
export ROS\_MASTER\_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS\_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}

```
9. Save the file and exit the nano editor.
10. Apply changes with the command below.

```
$ source ~/.bashrc

```

### NEW LDS-02 Configuration

The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

1. Install the LDS-02 driver and update TurtleBot3 package.

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/catkin_ws/src
$ git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/catkin_ws/src/turtlebot3 && git pull
$ rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ cd ~/catkin_ws && catkin_make

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc

```
3. Apply changes with the command below.

```
$ source ~/.bashrc

```

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.
* In order to use the webOS Robotics Platform, please refer to [webOS Robotics Platform](https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions "https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions") instruction. Packages will be cross-compiled using OpenEmbedded on a higher performance PC and an image file is created.

TurtleBot3 Hardware is compatible with Jetson Nano SBC.  

Please refer to the video below in order to set up the Jetson Nano for TurtleBot3.  

The [Jetson Nano Developer Kit setup](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit "https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit") must be completed first.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

### [Download TurtleBot3 SBC Image](#download-turtlebot3-sbc-image "#download-turtlebot3-sbc-image")

Download the correct image file for your hardware and ROS version.  

Melodic version images are created based on Ubuntu 18.04.

[![](/assets/images/icon_download.png)](https://www.robotis.com/service/download.php?no=2011 "https://www.robotis.com/service/download.php?no=2011")

**SHA256** : 312e1a5ad78447b901ae401ba31b2aaf94c1c760bdcafc60e2312df14e342640

[![](/assets/images/icon_download.png)](https://www.robotis.com/service/download.php?no=2065 "https://www.robotis.com/service/download.php?no=2065")

**SHA256** : 676bbcfc27fc6990bdf1e026247008f0525d344ccfaa106dca6c53d0bf7f4de8

* Please note that this image may not compatible with Raspberry Pi 4B with 8GB RAM.

[![](/assets/images/icon_download.png)](https://www.robotis.com/service/download.php?no=1905 "https://www.robotis.com/service/download.php?no=1905")

**SHA256** : 73546c63d3056bfc5538acc187f54dab6c1601096df320e60e0842bcb1b03d34

* ROS Melodic recovery image based on Ubuntu 18.04 above is recommended.
* Please note that this image may not compatible with Raspberry Pi 4B with 8GB RAM.

The recovery image files can be modified without a prior notice.

### [Unzip the downloaded image file](#unzip-the-downloaded-image-file "#unzip-the-downloaded-image-file")

Extract the `.img` file and save it in the local disk.

### Burn the image file

You can use various image burning tools.  

For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used.  

Choose your preferred tool to burn the image to microSD.

#### Raspberry Pi Imager

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png) **Download** Raspberry Pi Imager from raspberrypi.org](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)

1. Click `CHOOSE OS`.
2. Click `Use custom` and select the extracted `.img` file from local disk.
3. Click `CHOOSE STORAGE` and select the microSD.
4. Click `WRITE` to start burning the image.

#### Disks Utility

`Disks` utility is included in the recent Ubuntu Desktop. Search for “Disks” and launch the app.

![](/assets/images/platform/turtlebot3/setup/disks.gif)

1. Select the microSD card in the left panel.
2. Select `Restore Disk Image` option.
3. Open the `.img` file from local disk.
4. Click `Start Restoring...` > `Restore` button.

### Resize the Partition

In order to reduce the size of recovery image file and to decrease the time to burn the image onto microSD, the recovery partition is minimized.  

Please resize the partition to use the unallocated space.

**Be aware of selecting an incorrect disk or a partition. Partitioning a system disk of your PC may cause a serious system malfunction.**

[![](/assets/images/icon_download.png) Download or install GParted GUI tool](https://gparted.org/download.php "https://gparted.org/download.php")

![](/assets/images/platform/turtlebot3/setup/gparted.gif)

1. Select microSD card from the menu (mounted location may vary by system).
2. Right click on the yellow partition.
3. Select `Resize/Move` option.
4. Drag the right edge of the partition to all the way to the right end.
5. Click `Resize/Move` button.
6. Click the `Apply All Operations` green check button at the top.

### Configure the WiFi Network Setting

**NOTE** : If you encounter apt failures about the ROS GPG key (due to the existing GPG expiration), you may need to update GPG key. Please see [ROS GPG Key Expiration Incident](#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669 "#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669"), and proceed to the given solution.

1. Open a terminal window with `Alt`+`Ctrl`+`T` and go to the netplan directory in the microSD card.  

Start editing the `50-cloud-init.yaml` file with a superuser permission `sudo`.

```
$ cd /media/$USER/writable/etc/netplan
$ sudo nano 50-cloud-init.yaml

```

When the editor is opened, replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.  

![](/assets/images/platform/turtlebot3/setup/network_setup.gif)

If “No such file or directory” is returned, make sure the microSD is mounted to the system.

1. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.  

 e. Login with ID `ubuntu` and PASSWORD `turtlebot`.

HDMI cable has to be connected before powering the Raspberry Pi, or else the HDMI port of the Raspberry Pi will be disabled.

### ROS Network Configuration

Please follow the instructions below on the **SBC (Raspberry Pi)**.

1. Confirm the WiFi IP address.

```
$ ifconfig

```
2. Edit the `.bashrc` file.

```
$ nano ~/.bashrc

```
3. Find the `ROS_MASTER_URI` and `ROS_HOSTNAME` setting section, then modify the IP adddresses accordingly.

```
export ROS\_MASTER\_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS\_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}

```
4. Save the file with `Ctrl` + `S` and exit the nano editor with `Ctrl` + `X`.
5. Apply changes with the command below.

```
$ source ~/.bashrc

```

![](/assets/images/platform/turtlebot3/setup/ros1_sbc_netcfg.gif)

### NEW LDS-02 Configuration

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

1. Install the LDS-02 driver and update TurtleBot3 package

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/catkin_ws/src
$ git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/catkin_ws/src/turtlebot3 && git pull
$ rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ cd ~/catkin_ws && catkin_make

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc
$ source ~/.bashrc

```

**This is it! Now you are done with SBC setup :)**  

Next Step : [OpenCR Setup](/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup "/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup")

![](/assets/images/icon_unfold.png) **Click to expand : Manual SBC Setup Instructions**

Please be aware that this manual setup takes a lot more time than burning the recovery image file, but allows flexible choice of package installation. **This instruction is not recommended for the beginners**.

1. ![](/assets/images/icon_download.png) Download the `ubuntu-18.04.4-preinstalled-server-arm64+raspi3.img.xz` image on your PC.
	* [Ubuntu 18.04.4 Preinstalled Server ARM64 for Raspberry Pi3](http://old-releases.ubuntu.com/releases/18.04.4/ "http://old-releases.ubuntu.com/releases/18.04.4/")
2. Extract the downloaded file.
3. Burn the `.img` file to the microSD card. You can use various image burning tools.  

 For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used. Choose your preferred tool to burn the image to microSD.  

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)  

 a. Click `CHOOSE OS`.  

 b. Click `Use custom` and select the extracted `.img` file from local disk.  

 c. Click `CHOOSE STORAGE` and select the microSD.  

 d. Click `WRITE` to start burning the image.
4. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.
5. Configure the Raspberry Pi  

 a. Log in with default username(`ubuntu`) and password(`ubuntu`). After logged in, system will ask you to change the password.  

 b. Open automatic update setting file.

```
$ sudo nano /etc/apt/apt.conf.d/20auto-upgrades

```
6. Change the update settings as below.

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";

```

a. Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
7. Enter below command to configure the WiFi network setting.

```
$ sudo nano /etc/netplan/50-cloud-init.yaml

```
8. When the editor is opened, append below contents at the end of the file.  

 Replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

 Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
9. Reboot the Raspberry Pi.

```
$ sudo reboot

```
10. Set the `systemd` to prevent boot-up delay even if there is no network at startup. Run the command below to set mask the `systemd` process using the following command.

```
$ systemctl mask systemd-networkd-wait-online.service

```
11. Disable Suspend and Hibernation

```
$ sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

```
12. After rebooting the Raspberry Pi, if you wish to work from the Remote PC using SSH, use below command from the remote PC terminal. The default password is **ubuntu**.

```
$ ssh ubuntu@{IP Address of Raspberry PI}

```
13. Install ROS Melodic Morenia
Enter below commands to the terminal one at a time.  

In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic_rpi.sh "https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic_rpi.sh").

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic_rpi.sh
$ chmod 755 ./install_ros_melodic_rpi.sh
$ bash ./install_ros_melodic_rpi.sh

```

If the above installation fails, please refer to [the official ROS Melodic installation guide](http://wiki.ros.org/melodic/Installation/Ubuntu "http://wiki.ros.org/melodic/Installation/Ubuntu").
14. Install and Build ROS Packages.

```
$ sudo apt install ros-melodic-rosserial-python ros-melodic-tf
$ mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
$ sudo apt install ros-melodic-hls-lfcd-lds-driver
$ sudo apt install ros-melodic-turtlebot3-msgs
$ sudo apt install ros-melodic-dynamixel-sdk
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws/src/turtlebot3
$ rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ cd ~/catkin_ws/
$ echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ cd ~/catkin_ws && catkin_make -j1
$ echo 'source ~/catkin\_ws/devel/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

```
15. USB Port Setting

```
$ rosrun turtlebot3_bringup create_udev_rules

```
16. ROS Network Configuration
Confirm the WiFi IP address and edit the `.bashrc` file

```
$ nano ~/.bashrc

```
17. Modify the IP adddresses of `ROS_MASTER_URI` and the `ROS_HOSTNAME`.

```
export ROS\_MASTER\_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS\_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}

```
18. Save the file and exit the nano editor.
19. LDS Configuration
The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/catkin_ws/src
$ git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/catkin_ws && catkin_make

```
20. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-01' >> ~/.bashrc
$ source ~/.bashrc

```
21. Apply changes with the command below.

```
$ source ~/.bashrc

```

Please refer to the Ubuntu Blog below for more useful information.

* [Improving Security with Ubuntu](https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu "https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu")
* [Improving User Experience of TurtleBot3 Waffle Pi](https://ubuntu.com/blog/building-a-better-turtlebot3 "https://ubuntu.com/blog/building-a-better-turtlebot3")
* [How to set up TurtleBot3 Waffle Pi in minutes with Snaps](https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps "https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps")

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

### [Download TurtleBot3 SBC Image](#download-turtlebot3-sbc-image "#download-turtlebot3-sbc-image")

Download the correct image file for your hardware and ROS version.  

Noetic version images are created based on Ubuntu 20.04.

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 3B+` ROS Noetic image](https://www.robotis.com/service/download.php?no=2008 "https://www.robotis.com/service/download.php?no=2008")

**SHA256** : a7c57e20f2ee4204c95315866f4a274886094f7c63ed390b6d06d95074830309

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 4B (2GB or 4GB)` ROS Noetic image](https://www.robotis.com/service/download.php?no=2066 "https://www.robotis.com/service/download.php?no=2066")

**SHA256** : 9d48925a78381885916a6f3bb77891adbfae2b271b05fe2ae9a9b7ebd12c46cc

* Please note that this image may not compatible with Raspberry Pi 4B with 8GB RAM.

The recovery image files can be modified without a prior notice.

### [Unzip the downloaded image file](#unzip-the-downloaded-image-file "#unzip-the-downloaded-image-file")

Extract the `.img` file and save it in the local disk.

### Burn the image file

You can use various image burning tools.  

For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used.  

Choose your preferred tool to burn the image to microSD.

#### Raspberry Pi Imager

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png) **Download** Raspberry Pi Imager from raspberrypi.org](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)

1. Click `CHOOSE OS`.
2. Click `Use custom` and select the extracted `.img` file from local disk.
3. Click `CHOOSE STORAGE` and select the microSD.
4. Click `WRITE` to start burning the image.

#### Disks Utility

`Disks` utility is included in the recent Ubuntu Desktop. Search for “Disks” and launch the app.

![](/assets/images/platform/turtlebot3/setup/disks.gif)

1. Select the microSD card in the left panel.
2. Select `Restore Disk Image` option.
3. Open the `.img` file from local disk.
4. Click `Start Restoring...` > `Restore` button.

### Resize the Partition

In order to reduce the size of recovery image file and to decrease the time to burn the image onto microSD, the recovery partition is minimized.  

Please resize the partition to use the unallocated space.

**Be aware of selecting an incorrect disk or a partition. Partitioning a system disk of your PC may cause a serious system malfunction.**

[![](/assets/images/icon_download.png) Download or install GParted GUI tool](https://gparted.org/download.php "https://gparted.org/download.php")

![](/assets/images/platform/turtlebot3/setup/gparted.gif)

1. Select microSD card from the menu (mounted location may vary by system).
2. Right click on the yellow partition.
3. Select `Resize/Move` option.
4. Drag the right edge of the partition to all the way to the right end.
5. Click `Resize/Move` button.
6. Click the `Apply All Operations` green check button at the top.

### Configure the WiFi Network Setting

1. Open a terminal window with `Alt`+`Ctrl`+`T` and go to the netplan directory in the microSD card.  

Start editing the `50-cloud-init.yaml` file with a superuser permission `sudo`.

```
$ cd /media/$USER/writable/etc/netplan
$ sudo nano 50-cloud-init.yaml

```

When the editor is opened, replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.  

![](/assets/images/platform/turtlebot3/setup/network_setup.gif)

If “No such file or directory” is returned, make sure the microSD is mounted to the system.

1. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.  

 e. Login with ID `ubuntu` and PASSWORD `turtlebot`.

HDMI cable has to be connected before powering the Raspberry Pi, or else the HDMI port of the Raspberry Pi will be disabled.

### ROS Network Configuration

**NOTE** : If you encounter apt failures about the ROS GPG key (due to the existing GPG expiration), you may need to update GPG key. Please see [ROS GPG Key Expiration Incident](#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669 "#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669"), and proceed to the given solution.

Please follow the instructions below on the **SBC (Raspberry Pi)**.

1. Confirm the WiFi IP address.

```
$ ifconfig

```
2. Edit the `.bashrc` file.

```
$ nano ~/.bashrc

```
3. Find the `ROS_MASTER_URI` and `ROS_HOSTNAME` setting section, then modify the IP adddresses accordingly.

```
export ROS\_MASTER\_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS\_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}

```
4. Save the file with `Ctrl` + `S` and exit the nano editor with `Ctrl` + `X`.
5. Apply changes with the command below.

```
$ source ~/.bashrc

```

![](/assets/images/platform/turtlebot3/setup/ros1_sbc_netcfg.gif)

### NEW LDS-02 Configuration

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

1. Install the LDS-02 driver and update TurtleBot3 package

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/catkin_ws/src
$ git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/catkin_ws/src/turtlebot3 && git pull
$ rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ cd ~/catkin_ws && catkin_make

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc
$ source ~/.bashrc

```

**This is it! Now you are done with SBC setup :)**  

Next Step : [OpenCR Setup](/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup "/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup")

![](/assets/images/icon_unfold.png) **Click to expand : Manual SBC Setup Instructions**

Please be aware that this manual setup takes a lot more time than burning the recovery image file, but allows flexible choice of package installation. **This instruction is not recommended for the beginners**.

1. ![](/assets/images/icon_download.png) Download the proper `Ubuntu 20.04.1(Focal) Preinstalled Server` image on your PC.
	* [Ubuntu 20.04.1(Focal) Preinstalled Server for Raspberry Pi3(arm64)](http://cdimage.ubuntu.com/ubuntu-server/focal/daily-preinstalled/current/ "http://cdimage.ubuntu.com/ubuntu-server/focal/daily-preinstalled/current/")
2. Extract the downloaded file.
3. Burn the `.img` file to the microSD card. You can use various image burning tools.  

 For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used. Choose your preferred tool to burn the image to microSD.  

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)  

 a. Click `CHOOSE OS`.  

 b. Click `Use custom` and select the extracted `.img` file from local disk.  

 c. Click `CHOOSE STORAGE` and select the microSD.  

 d. Click `WRITE` to start burning the image.
4. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.
5. Configure the Raspberry Pi  

 a. Log in with default username(`ubuntu`) and password(`ubuntu`). After logged in, system will ask you to change the password.  

 b. Open automatic update setting file.

```
$ sudo nano /etc/apt/apt.conf.d/20auto-upgrades

```
6. Change the update settings as below.

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";

```

a. Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
7. Enter below command to configure the WiFi network setting.

```
$ sudo nano /etc/netplan/50-cloud-init.yaml

```
8. When the editor is opened, append below contents at the end of the file.  

 Replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

 Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
9. Reboot the Raspberry Pi.

```
$ sudo reboot

```
10. Set the `systemd` to prevent boot-up delay even if there is no network at startup. Run the command below to set mask the `systemd` process using the following command.

```
$ systemctl mask systemd-networkd-wait-online.service

```
11. Disable Suspend and Hibernation

```
$ sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

```
12. After rebooting the Raspberry Pi, if you wish to work from the Remote PC using SSH, use below command from the remote PC terminal. The default password is **ubuntu**.

```
$ ssh ubuntu@{IP Address of Raspberry PI}

```
13. Install ROS Noetic Ninjemys
Enter below commands to the terminal one at a time.  

In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic_rpi.sh "https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic_rpi.sh").

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic_rpi.sh
$ chmod 755 ./install_ros_noetic_rpi.sh
$ bash ./install_ros_noetic_rpi.sh

```

If the above installation fails, please refer to [the official ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu "http://wiki.ros.org/noetic/Installation/Ubuntu").
14. Install and Build ROS Packages.

```
$ sudo apt install ros-noetic-rosserial-python ros-noetic-tf
$ sudo apt install ros-noetic-hls-lfcd-lds-driver
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-dynamixel-sdk
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws/src/turtlebot3
$ rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ cd ~/catkin_ws/
$ catkin_make -j1
$ source ~/.bashrc

```
15. USB Port Setting

```
$ rosrun turtlebot3_bringup create_udev_rules

```
16. ROS Network Configuration
Confirm the WiFi IP address and edit the `.bashrc` file

```
$ nano ~/.bashrc

```
17. Modify the IP adddresses of `ROS_MASTER_URI` and the `ROS_HOSTNAME`.

```
export ROS\_MASTER\_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS\_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}

```
18. Save the file and exit the nano editor.
19. LDS Configuration
The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/catkin_ws/src
$ git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/catkin_ws && catkin_make

```
20. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-01' >> ~/.bashrc
$ source ~/.bashrc

```
21. Apply changes with the command below.

```
$ source ~/.bashrc

```

Please refer to the Ubuntu Blog below for more useful information.

* [Improving Security with Ubuntu](https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu "https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu")
* [Improving User Experience of TurtleBot3 Waffle Pi](https://ubuntu.com/blog/building-a-better-turtlebot3 "https://ubuntu.com/blog/building-a-better-turtlebot3")
* [How to set up TurtleBot3 Waffle Pi in minutes with Snaps](https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps "https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps")

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* This SBC Setup section is specifically written for **Raspberry Pi 3B+** which is the current official TurtleBot3 SBC.
* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.
* In order to use the webOS Robotics Platform, please refer to [webOS Robotics Platform](https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions "https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions") instruction. Packages will be cross-compiled using OpenEmbedded on a higher performance PC and an image file is created.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

### [Download SBC OS Image](#download-sbc-os-image "#download-sbc-os-image")

Download the correct image file for your hardware and ROS version.  

ROS2 Dashing requires Ubuntu 18.04.

[![](/assets/images/icon_download.png) **Download for Rasbperry Pi 4** `ubuntu-18.04.4-preinstalled-server-arm64+raspi4.img.xz` OS image](http://old-releases.ubuntu.com/releases/18.04.4/ "http://old-releases.ubuntu.com/releases/18.04.4/")

[![](/assets/images/icon_download.png) **Download for Raspberry Pi 3B+** `ubuntu-18.04.3-preinstalled-server-arm64+raspi3.img.xz` OS image](http://old-releases.ubuntu.com/releases/18.04.3/ "http://old-releases.ubuntu.com/releases/18.04.3/")

### [Unzip the downloaded image file](#unzip-the-downloaded-image-file "#unzip-the-downloaded-image-file")

Extract the `.img` file and save it in the local disk.

### Burn the image file

You can use various image burning tools.  

For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used.  

Choose your preferred tool to burn the image to microSD.

#### Raspberry Pi Imager

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png) **Download** Raspberry Pi Imager from raspberrypi.org](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)

1. Click `CHOOSE OS`.
2. Click `Use custom` and select the extracted `.img` file from local disk.
3. Click `CHOOSE STORAGE` and select the microSD.
4. Click `WRITE` to start burning the image.

#### Disks Utility

`Disks` utility is included in the recent Ubuntu Desktop. Search for “Disks” and launch the app.

![](/assets/images/platform/turtlebot3/setup/disks.gif)

1. Select the microSD card in the left panel.
2. Select `Restore Disk Image` option.
3. Open the `.img` file from local disk.
4. Click `Start Restoring...` > `Restore` button.

### Boot Up the Raspberry Pi

1. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.
2. Connect input devices to the USB port of Raspberry Pi
3. Insert the microSD card.
4. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.
5. Login with ID `ubuntu` and PASSWORD `ubuntu`.  

 Once logged in, the system will request to change the password.

HDMI cable has to be connected before powering the Raspberry Pi, or else the HDMI port of the Raspberry Pi will be disabled.

### Configure the Raspberry Pi

1. Open automatic update setting file.

```
$ sudo nano /etc/apt/apt.conf.d/20auto-upgrades

```
2. Edit to disable automatic update settings as below.

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";

```

Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
3. Enter below command to configure the WiFi network setting.

```
$ sudo nano /etc/netplan/50-cloud-init.yaml

```
4. When the editor is opened, append below contents at the end of the file.  

 Replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

 Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
5. Apply all configuration for the renderers, and then reboot the Raspberry Pi.

```
$ sudo netplan apply
$ reboot

```
6. Set the `systemd` to prevent boot-up delay even if there is no network at startup. Run the command below to set mask the `systemd` process using the following command.

```
$ systemctl mask systemd-networkd-wait-online.service

```
7. Disable Suspend and Hibernation

```
$ sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

```
8. Install and enable the SSH

```
$ sudo apt install ssh
$ sudo systemctl enable --now ssh
$ reboot

```
9. After rebooting the Raspberry Pi, if you wish to work from the Remote PC using SSH, use below command from the remote PC terminal. The default password is **ubuntu**.

```
$ ssh ubuntu@{IP Address of Raspberry PI}

```

### [Add Swap Space](#add-swap-space "#add-swap-space")

1. Enter the command below to create 2GB swap space.

```
$ sudo swapoff /swapfile
$ sudo fallocate -l 2G /swapfile
$ sudo chmod 600 /swapfile
$ sudo mkswap /swapfile
$ sudo swapon /swapfile
$ sudo nano /etc/fstab

```

You can ignore below error when entering `swapoff /swapfile` command.

```
swapoff: /swapfile: swapoff failed: No such file or directory

```
2. When the editor opens the fstab file, append below contents at the end of the file.

```
/swapfile swap swap defaults 0 0

```
3. Check if 2GB of swap space is correctly configured.

```
$ sudo free -h
              total        used        free      shared  buff/cache   available
Mem:           912M         97M        263M        4.4M        550M        795M
Swap:          2.0G          0B        2.0G

```

Please refer to the Ubuntu Blog below for more useful information.

* [Improving Security with Ubuntu](https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu "https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu")
* [Improving User Experience of TurtleBot3 Waffle Pi](https://ubuntu.com/blog/building-a-better-turtlebot3 "https://ubuntu.com/blog/building-a-better-turtlebot3")
* [How to set up TurtleBot3 Waffle Pi in minutes with Snaps](https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps "https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps")

### Install ROS 2 Dashing Diademata

**Reference**: [Official ROS 2 Dashing Installation Guide](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/ "https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/")

1. Open a terminal on **SBC**
2. Setup locale

```
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC\_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8

```
3. Setup sources

```
$ sudo apt update && sudo apt install curl gnupg2 lsb-release
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

```
4. Install ROS 2 packages

```
$ sudo apt update
$ sudo apt install ros-dashing-ros-base

```
5. Install and Build ROS Packages.

```
$ sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
$ sudo apt install ros-dashing-hls-lfcd-lds-driver
$ sudo apt install ros-dashing-turtlebot3-msgs
$ sudo apt install ros-dashing-dynamixel-sdk
$ mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
$ git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/turtlebot3_ws/src/turtlebot3
$ rm -r turtlebot3_cartographer turtlebot3_navigation2
$ cd ~/turtlebot3_ws/
$ echo 'source /opt/ros/dashing/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ colcon build --symlink-install --parallel-workers 1
$ echo 'source ~/turtlebot3\_ws/install/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

```
6. USB Port Setting for OpenCR

```
$ sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger

```
7. ROS Domain ID Setting
In ROS2 DDS communication, `ROS_DOMAIN_ID` must be matched between **Remote PC** and **TurtleBot3** for communication under the same network environment. Following commands shows how to assign a `ROS_DOMAIN_ID` to SBC in TurtleBot3.
	* A default ID of **TurtleBot3** is `30`.
	* Configuring the `ROS_DOMAIN_ID` of Remote PC and SBC in TurtleBot3 to `30` is recommended.

```
$ echo 'export ROS\_DOMAIN\_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc

```

**WARNING** : Do not use an identical ROS\_DOMAIN\_ID with others in the same network. It will cause a conflict of communication between users under the same network environment.

### NEW LDS-02 Configuration

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

1. Install the LDS-02 driver and update TurtleBot3 package

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/turtlebot3_ws/src
$ git clone -b dashing-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/turtlebot3_ws && colcon build --symlink-install

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc
$ source ~/.bashrc

```

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.
* In order to use the webOS Robotics Platform, please refer to [webOS Robotics Platform](https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions "https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions") instruction. Packages will be cross-compiled using OpenEmbedded on a higher performance PC and an image file is created.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

### [Download TurtleBot3 SBC Image](#download-turtlebot3-sbc-image "#download-turtlebot3-sbc-image")

Download the correct image file for your hardware and ROS version.  

Foxy version images are created based on Ubuntu 20.04.

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 3B+` ROS2 Foxy image](https://www.robotis.com/service/download.php?no=2058 "https://www.robotis.com/service/download.php?no=2058")

**SHA256** : e1916b75573e3944c72552664ee1e32e9be32a026bd5b4323d0a4b5778243a1e

[![](/assets/images/icon_download.png) **Download** `Raspberry Pi 4B (2GB or 4GB)` ROS2 Foxy image](https://www.robotis.com/service/download.php?no=2064 "https://www.robotis.com/service/download.php?no=2064")

**SHA256** : 8b8b54ad80c7a02ae35da8e9e5d9750fdf21ec6098052a804986ab22ce10ba7e

* Please note that this image may not compatible with Raspberry Pi 4B with 8GB RAM.

The recovery image files can be modified without a prior notice.

### [Unzip the downloaded image file](#unzip-the-downloaded-image-file "#unzip-the-downloaded-image-file")

Extract the `.img` file and save it in the local disk.

### Burn the image file

You can use various image burning tools.  

For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used.  

Choose your preferred tool to burn the image to microSD.

#### Raspberry Pi Imager

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png) **Download** Raspberry Pi Imager from raspberrypi.org](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)

1. Click `CHOOSE OS`.
2. Click `Use custom` and select the extracted `.img` file from local disk.
3. Click `CHOOSE STORAGE` and select the microSD.
4. Click `WRITE` to start burning the image.

#### Disks Utility

`Disks` utility is included in the recent Ubuntu Desktop. Search for “Disks” and launch the app.

![](/assets/images/platform/turtlebot3/setup/disks.gif)

1. Select the microSD card in the left panel.
2. Select `Restore Disk Image` option.
3. Open the `.img` file from local disk.
4. Click `Start Restoring...` > `Restore` button.

### Resize the Partition

In order to reduce the size of recovery image file and to decrease the time to burn the image onto microSD, the recovery partition is minimized.  

Please resize the partition to use the unallocated space.

**Be aware of selecting an incorrect disk or a partition. Partitioning a system disk of your PC may cause a serious system malfunction.**

[![](/assets/images/icon_download.png) Download or install GParted GUI tool](https://gparted.org/download.php "https://gparted.org/download.php")

![](/assets/images/platform/turtlebot3/setup/gparted.gif)

1. Select microSD card from the menu (mounted location may vary by system).
2. Right click on the yellow partition.
3. Select `Resize/Move` option.
4. Drag the right edge of the partition to all the way to the right end.
5. Click `Resize/Move` button.
6. Click the `Apply All Operations` green check button at the top.

### Configure the WiFi Network Setting

**NOTE** : If you encounter apt failures about the ROS GPG key (due to the existing GPG expiration), you may need to update GPG key. Please see [ROS GPG Key Expiration Incident](#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669 "#https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669"), and proceed to the given solution.

1. Open a terminal window with `Alt`+`Ctrl`+`T` and go to the netplan directory in the microSD card.  

Start editing the `50-cloud-init.yaml` file with a superuser permission `sudo`.

```
$ cd /media/$USER/writable/etc/netplan
$ sudo nano 50-cloud-init.yaml

```

When the editor is opened, replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.  

![](/assets/images/platform/turtlebot3/setup/network_setup.gif)

If “No such file or directory” is returned, make sure the microSD is mounted to the system.

1. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.  

 e. Login with ID `ubuntu` and PASSWORD `turtlebot`.

HDMI cable has to be connected before powering the Raspberry Pi, or else the HDMI port of the Raspberry Pi will be disabled.

### ROS2 Network Configuration

In ROS2 DDS communication, `ROS_DOMAIN_ID` must be matched between **Remote PC** and **TurtleBot3** for communication under the same network environment.  

The default ROS Domain ID for TurtleBot3 is set to `30` in the ***.bashrc*** file.  

Please modify the ID to avoid any conflict when there are identical ID in the same network.

```
ROS_DOMAIN_ID=30 #TURTLEBOT3

```

**WARNING** : Do not use an identical ROS\_DOMAIN\_ID with others in the same network. It will cause a conflict of communication between users under the same network environment.

### NEW LDS-02 Configuration

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

1. Install the LDS-02 driver and update TurtleBot3 package

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/turtlebot3_ws/src
$ git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/turtlebot3_ws/src/turtlebot3 && git pull
$ rm -r turtlebot3_cartographer turtlebot3_navigation2
$ cd ~/turtlebot3_ws && colcon build --symlink-install

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc
$ source ~/.bashrc

```

**This is it! Now you are done with SBC setup :)**  

Next Step : [OpenCR Setup](/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup "/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup")

![](/assets/images/icon_unfold.png) **Click to expand : Manual SBC Setup Instructions**

Please be aware that this manual setup takes a lot more time than burning the recovery image file, but allows flexible choice of package installation. **This instruction is not recommended for the beginners**.

1. ![](/assets/images/icon_download.png) Download the latest `Ubuntu 20.04 server` image for your SBC from the link below.
	* [Ubuntu 20.04 Server 64-bit](https://ubuntu.com/download/raspberry-pi "https://ubuntu.com/download/raspberry-pi")
2. Extract the downloaded file.
3. Burn the `.img` file to the microSD card. You can use various image burning tools.  

 For example, `Raspberry Pi Imager` or Linux `Disks` utility can be used. Choose your preferred tool to burn the image to microSD.  

![](/assets/images/platform/turtlebot3/setup/rpi_imager.gif)  

 a. Click `CHOOSE OS`.  

 b. Click `Use custom` and select the extracted `.img` file from local disk.  

 c. Click `CHOOSE STORAGE` and select the microSD.  

 d. Click `WRITE` to start burning the image.
4. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.
5. Configure the Raspberry Pi  

 a. Log in with default username(`ubuntu`) and password(`ubuntu`). After logged in, system will ask you to change the password.  

 b. Open automatic update setting file.

```
$ sudo nano /etc/apt/apt.conf.d/20auto-upgrades

```
6. Change the update settings as below.

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";

```

a. Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
7. Enter below command to configure the WiFi network setting.

```
$ sudo nano /etc/netplan/50-cloud-init.yaml

```
8. When the editor is opened, enter below contents to the file.  

 Replace the `WIFI_SSID` and `WIFI_PASSWORD` with your wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)  

 Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
9. Reboot the Raspberry Pi.

```
$ sudo reboot

```
10. Set the `systemd` to prevent boot-up delay even if there is no network at startup. Run the command below to set mask the `systemd` process using the following command.

```
$ systemctl mask systemd-networkd-wait-online.service

```
11. Disable Suspend and Hibernation

```
$ sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

```
12. After rebooting the Raspberry Pi, if you wish to work from the Remote PC using SSH, use below command from the remote PC terminal. Make sure to use the password you set in `Step 5`.

```
$ ssh ubuntu@{IP Address of Raspberry PI}

```
13. Install ROS2 Foxy Fiztroy  

Enter below commands to the Raspberry Pi terminal one at a time.  

In order to check the details of the easy installation script, please refer to [the script file](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy_rpi.sh "https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy_rpi.sh").

```
$ sudo apt update
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy_rpi.sh
$ chmod 755 ./install_ros2_foxy_rpi.sh
$ bash ./install_ros2_foxy_rpi.sh

```

If the above installation fails, please refer to [the official ROS2 Foxy installation guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html "https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html").
14. Install and Build ROS Packages.

```
$ sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
$ sudo apt install ros-foxy-hls-lfcd-lds-driver
$ sudo apt install ros-foxy-turtlebot3-msgs
$ sudo apt install ros-foxy-dynamixel-sdk
$ mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/turtlebot3_ws/src/turtlebot3
$ rm -r turtlebot3_cartographer turtlebot3_navigation2
$ cd ~/turtlebot3_ws/
$ echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ colcon build --symlink-install --parallel-workers 1
$ echo 'source ~/turtlebot3\_ws/install/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

```
15. USB Port Setting for OpenCR

```
$ sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger

```
16. ROS Domain ID Setting
In ROS2 DDS communication, `ROS_DOMAIN_ID` must be matched between **Remote PC** and **TurtleBot3** for communication under the same network environment. Following commands shows how to assign a `ROS_DOMAIN_ID` to SBC in TurtleBot3.
	* A default ID of **TurtleBot3** is `30`.
	* Configuring the `ROS_DOMAIN_ID` of Remote PC and SBC in TurtleBot3 to `30` is recommended.

```
$ echo 'export ROS\_DOMAIN\_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc

```

**WARNING** : Do not use an identical ROS\_DOMAIN\_ID with others in the same network. It will cause a conflict of communication between users under the same network environment.

1. LDS Configuration
The TurtleBot3 LDS has been updated to LDS-02 since 2022 models.  

Please follow the instructions below on the **SBC (Raspberry Pi)** of TurtleBot3.

```
$ sudo apt update
$ sudo apt install libudev-dev
$ cd ~/turtlebot3_ws/src
$ git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/turtlebot3_ws && colcon build --symlink-install

```
2. Export the LDS\_MODEL to the bashrc file. Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc

```
3. Apply changes with the command below.

```
$ source ~/.bashrc

```

Please refer to the Ubuntu Blog below for more useful information.

* [Improving Security with Ubuntu](https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu "https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu")
* [Improving User Experience of TurtleBot3 Waffle Pi](https://ubuntu.com/blog/building-a-better-turtlebot3 "https://ubuntu.com/blog/building-a-better-turtlebot3")
* [How to set up TurtleBot3 Waffle Pi in minutes with Snaps](https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps "https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps")

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* This process may take long time. Please do not use battery while following this section.
* An HDMI monitor and input devices such as a keyboard and a mouse will be required to complete this setup.
* In order to use the webOS Robotics Platform, please refer to [webOS Robotics Platform](https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions "https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions") instruction. Packages will be cross-compiled using OpenEmbedded on a higher performance PC and an image file is created.

### [Prepare microSD Card and Reader](#prepare-microsd-card-and-reader "#prepare-microsd-card-and-reader")

If you PC do not have a microSD slot, please use a microSD card reader to burn the recovery image.  

![](/assets/images/platform/turtlebot3/setup/micro_sd_reader.png)

The microSD card reader is not included in the TurtleBot3 package.

### Install Raspberry Pi Imager

Download the `Raspberry Pi Imager` to install Ubuntu Server 22.04 for Raspberry Pi.  

If the Raspberry Pi Imager is already installed, update to the latest version.  

Please refer to [this article](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/ "https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/") to find more information about Raspberry Pi Imager.

[![](/assets/images/icon_download.png) **Download** Raspberry Pi Imager from raspberrypi.org](https://www.raspberrypi.org/software/ "https://www.raspberrypi.org/software/")

### Install Ubuntu 22.04

1. Run Raspberry Pi Imager
2. Click `CHOOSE OS`.
3. Select `Other gerneral-purpose OS`.
4. Select `Ubuntu`.
5. Select `Ubuntu Server 22.04.5 LTS (64-bit)` that support RPi 3/4/400.
6. Click `CHOOSE STORAGE` and select the micro SD card.
7. Click `WRITE` to install the Ubuntu.

### Configure the Raspberry Pi

HDMI cable must be connected before powering the Raspberry Pi, or else the HDMI port of the Raspberry Pi will be disabled.

1. Boot Up the Raspberry Pi  

 a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.  

 b. Connect input devices to the USB port of Raspberry Pi.  

 c. Insert the microSD card.  

 d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.  

 e. Login with ID `ubuntu` and PASSWORD `ubuntu`. Once logged in, you’ll be asked to change the password.
2. Open the network configuration file with the command below.

```
$ sudo nano /writable/etc/netplan/50-cloud-init.yaml

```
3. When the editor is opened, edit the content as below while replacing the `WIFI_SSID` and `WIFI_PASSWORD` with your actual wifi SSID and password.  

![](/assets/images/platform/turtlebot3/setup/ros2_sbc_netcfg.png)
4. Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
5. Enter the command below to edit automatic update setting file.

```
$ sudo nano /etc/apt/apt.conf.d/20auto-upgrades

```
6. Change the update settings as below.

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";

```
7. Save the file with `Ctrl`+`S` and exit with `Ctrl`+`X`.
8. Set the `systemd` to prevent boot-up delay even if there is no network at startup. Run the command below to set mask the `systemd` process using the following command.

```
$ systemctl mask systemd-networkd-wait-online.service

```
9. Disable Suspend and Hibernation

```
$ sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

```
10. Reboot the Raspberry Pi.

```
$ reboot

```
11. After rebooting the Raspberry Pi, if you wish to work from the Remote PC using SSH, use below command from the remote PC terminal. Make sure to use the password you set in `Step 1`.

```
$ ssh ubuntu@{IP Address of Raspberry PI}

```
12. Install ROS2 Humble Hawksbill  

Follow the instruction in [the official ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html "https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html").
13. Install and Build ROS Packages.  

Building the `turtlebot3` package may take longer than an hour. Please use the SMPS to ensure the system is always powered.

```
$ sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
$ sudo apt install ros-humble-hls-lfcd-lds-driver
$ sudo apt install ros-humble-turtlebot3-msgs
$ sudo apt install ros-humble-dynamixel-sdk
$ sudo apt install libudev-dev
$ mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
$ git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
$ cd ~/turtlebot3_ws/src/turtlebot3
$ rm -r turtlebot3_cartographer turtlebot3_navigation2
$ cd ~/turtlebot3_ws/
$ echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ colcon build --symlink-install --parallel-workers 1
$ echo 'source ~/turtlebot3\_ws/install/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

```
14. USB Port Setting for OpenCR

```
$ sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger

```
15. ROS Domain ID Setting
In ROS2 DDS communication, `ROS_DOMAIN_ID` must be matched between **Remote PC** and **TurtleBot3** for communication under the same network environment. Following commands shows how to assign a `ROS_DOMAIN_ID` to SBC in TurtleBot3.
	* A default ID of **TurtleBot3** is `30`.
	* Configuring the `ROS_DOMAIN_ID` of Remote PC and SBC in TurtleBot3 to `30` is recommended.

```
$ echo 'export ROS\_DOMAIN\_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc

```

**WARNING** : Do not use an identical ROS\_DOMAIN\_ID with others in the same network. It will cause a conflict of communication between users under the same network environment.

### LDS Configuration

The TurtleBot3 LDS has been updated to LDS-02 since 2022.  

If you have purchased TurtleBot3 after 2022, please use `LDS-02` for the LDS\_MODEL.

| LDS-01 | LDS-02 |
| --- | --- |
|  |  |

Depending on your LDS model, use `LDS-01` or `LDS-02`.

```
$ echo 'export LDS\_MODEL=LDS-02' >> ~/.bashrc

```

Apply changes with the command below.

```
$ source ~/.bashrc

```

**This is it! Now you are done with SBC setup :)**  

Next Step : [OpenCR Setup](/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup "/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup")

Please refer to the Ubuntu Blog below for more useful information.

* [Improving Security with Ubuntu](https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu "https://ubuntu.com/blog/steps-to-maximise-robotics-security-with-ubuntu")
* [Improving User Experience of TurtleBot3 Waffle Pi](https://ubuntu.com/blog/building-a-better-turtlebot3 "https://ubuntu.com/blog/building-a-better-turtlebot3")
* [How to set up TurtleBot3 Waffle Pi in minutes with Snaps](https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps "https://ubuntu.com/blog/how-to-set-up-turtlebot3-in-minutes-with-snaps")

## [SBC Setup](#sbc-setup "#sbc-setup")

**WARNING**

* TurtleBot3 on Windows is running on a single PC assembled on TurtleBot3 instead of Raspberry Pi.
* If you have replaced Raspberry Pi with UP2 or Intel NUC and followed PC setup section on it, you can bypass this section.

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

