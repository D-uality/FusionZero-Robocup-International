$(function () {

    //首页nav
    $(".nav_list li").hover(function () {
        $(this).find('ul').stop(false, true).slideDown();
    }, function () {
        $(this).find('ul').stop(false, true).slideUp();
    });

    var swiperNav = new Swiper('.navSlide', {
        pagination: '.swiper-pagination',
        slidesPerView: 4,
        paginationClickable: true
    });

    //banner
    // $(".bannerPC").slide({mainCell:".bd ul",autoPlay:true});

    //首页手机banner
    var mySwiperBanMob = new Swiper('.bannerMob', {
        paginationClickable: true,
        pagination: '.swiper-paginationBanMob'
    });


    //首页new
    $(".newI").slide({ mainCell: ".bd ul" });

    //mob底部热线
    $(".f-close").click('touchstart', function () {
        $(this).parents(".foot-tel").hide();
    });


    /*左侧nav*/
    $("ul.rt-nav a").click(function (event) {
        if ($(this).siblings("ul").length > 0) {
            var a = $(this);
            var thisname = a.attr('class');
            if (thisname == null || thisname == 0) {
                a.siblings("ul").slideDown(300);
                a.parent().siblings().find('ul').slideUp(300).siblings("a").removeClass();
                a.siblings("i").removeClass('in');
                var parent = a.parent().parents("ul").attr('class');
                switch (parent) {
                    case "rt-nav":
                        a.addClass("box-on");
                        a.siblings('i').addClass('in');
                        a.siblings('i').text('-').parent().siblings().find('a').siblings('i').text('+');
                        break;
                    case "two":
                        a.addClass("box-on");
                        a.siblings('i').addClass('in');
                        a.siblings('i').text('-').parent().siblings().find('a').siblings('i').text('+');
                        break;
                    case "three":
                        a.addClass("box-on");
                        break;
                }
            } else {
                a.removeClass().siblings('ul').slideUp(300);
                a.siblings('ul').find('a').removeClass().siblings('ul').slideUp(300);
                a.siblings("i").removeClass('in');
                a.siblings('i').text('+');
            };
            return false;
        };
    });

    $(".menu,.nav-btn").click('touchstart', function () {
        $(this).parents("body").addClass("current");
        $(".right_navBox").addClass("current");
    });
    $(".que-name i").click('touchstart', function () {
        $(this).parents(".right_navBox").removeClass("current");
        $(this).parents("body").removeClass("current");
    });
    $(".navBoxBg").click('touchstart', function () {
        $(this).parent().removeClass("current");
        $(this).parents("body").removeClass("current");
    });

    // 滚回顶部
    $(".scroll_top").click(function () {
        $("html,body").animate({ scrollTop: '0px' }, 800);
    });


    //内页nav
    $("ul.detNav a").click(function (event) {
        if ($(this).siblings("ul").length > 0) {
            var a = $(this);
            var thisname = a.attr('class');
            if (thisname == null || thisname == 0) {
                a.siblings("ul").slideDown(300);
                a.parent().siblings().find('ul').slideUp(500).siblings("a").removeClass();
                var parent = a.parent().parents("ul").attr('class');
                switch (parent) {
                    case "detNav":
                        a.addClass("box-on");
                        break;
                    case "detTwo":
                        a.addClass("box-on");
                        break;
                    case "detThree":
                        a.addClass("box-on");
                        break;
                }
            } else {
                a.removeClass().siblings('ul').slideUp(300);
                a.siblings('ul').find('a').removeClass().siblings('ul').slideUp(300);
            };
            return false;
        };
    });


    //内页手机nav
    $("ul.detNavCont a").click(function (event) {
        if ($(this).siblings("ul").length > 0) {
            var a = $(this);
            var thisname = a.attr('class');
            if (thisname == null || thisname == 0) {
                a.siblings("ul").slideDown(300);
                a.parent().siblings().find('ul').slideUp(500).siblings("a").removeClass();
                var parent = a.parent().parents("ul").attr('class');
                switch (parent) {
                    case "detNavCont":
                        a.addClass("box-on");
                        break;
                    case "detTwoM":
                        a.addClass("box-in");
                        break;
                    case "detThreeM":
                        a.addClass("box-in");
                        break;
                }
            } else {
                a.removeClass().siblings('ul').slideUp(300);
                a.siblings('ul').find('a').removeClass().siblings('ul').slideUp(300);
            };
            return false;
        };
    });

    $(".classify").click(function () {
        $(".detNavMob").slideToggle();
    });

    //content-product
    $(".proSlide").slide({ mainCell: ".bd ul", autoPlay: true });

});

$(document).ready(function () {
    $('.detNav li, .detNavCont li').click(function(){
        $(this).find('ul').slideToggle('slow');
    })
});

$(function(){
	$('#owl-demo').owlCarousel({
        items: 4,
        autoPlay: 3000,
        lazyLoad: 2000,
        itemsMobile: [479,2],
	});
});

$(document).ready(function () {
 var swiper = new Swiper('.swiper-container', {
      spaceBetween: 30,
      centeredSlides: true,
      autoplay: {
        delay: 3000,
        disableOnInteraction: false,
      },
       // 如果需要分页器
      pagination: {
         el: '.swiper-paginationBanMob',
      },
    }); 
})    