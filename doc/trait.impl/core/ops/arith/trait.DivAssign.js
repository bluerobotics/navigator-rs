(function() {
    var implementors = Object.fromEntries([["palette",[["impl&lt;C: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>, T: <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a> for <a class=\"struct\" href=\"palette/blend/struct.PreAlpha.html\" title=\"struct palette::blend::PreAlpha\">PreAlpha</a>&lt;C, T&gt;"],["impl&lt;C: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>, T: <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a> for <a class=\"struct\" href=\"palette/struct.Alpha.html\" title=\"struct palette::Alpha\">Alpha</a>&lt;C, T&gt;"],["impl&lt;S, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a> for <a class=\"struct\" href=\"palette/luma/struct.Luma.html\" title=\"struct palette::luma::Luma\">Luma</a>&lt;S, T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.Component.html\" title=\"trait palette::Component\">Component</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,\n    S: <a class=\"trait\" href=\"palette/luma/trait.LumaStandard.html\" title=\"trait palette::luma::LumaStandard\">LumaStandard</a>&lt;TransferFn = <a class=\"struct\" href=\"palette/encoding/linear/struct.LinearFn.html\" title=\"struct palette::encoding::linear::LinearFn\">LinearFn</a>&gt;,</div>"],["impl&lt;S, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a> for <a class=\"struct\" href=\"palette/rgb/struct.Rgb.html\" title=\"struct palette::rgb::Rgb\">Rgb</a>&lt;S, T&gt;<div class=\"where\">where\n    S: <a class=\"trait\" href=\"palette/rgb/trait.RgbStandard.html\" title=\"trait palette::rgb::RgbStandard\">RgbStandard</a>&lt;TransferFn = <a class=\"struct\" href=\"palette/encoding/linear/struct.LinearFn.html\" title=\"struct palette::encoding::linear::LinearFn\">LinearFn</a>&gt;,\n    T: <a class=\"trait\" href=\"palette/trait.Component.html\" title=\"trait palette::Component\">Component</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,</div>"],["impl&lt;S, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;T&gt; for <a class=\"struct\" href=\"palette/luma/struct.Luma.html\" title=\"struct palette::luma::Luma\">Luma</a>&lt;S, T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.Component.html\" title=\"trait palette::Component\">Component</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,\n    S: <a class=\"trait\" href=\"palette/luma/trait.LumaStandard.html\" title=\"trait palette::luma::LumaStandard\">LumaStandard</a>&lt;TransferFn = <a class=\"struct\" href=\"palette/encoding/linear/struct.LinearFn.html\" title=\"struct palette::encoding::linear::LinearFn\">LinearFn</a>&gt;,</div>"],["impl&lt;S, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;T&gt; for <a class=\"struct\" href=\"palette/rgb/struct.Rgb.html\" title=\"struct palette::rgb::Rgb\">Rgb</a>&lt;S, T&gt;<div class=\"where\">where\n    S: <a class=\"trait\" href=\"palette/rgb/trait.RgbStandard.html\" title=\"trait palette::rgb::RgbStandard\">RgbStandard</a>&lt;TransferFn = <a class=\"struct\" href=\"palette/encoding/linear/struct.LinearFn.html\" title=\"struct palette::encoding::linear::LinearFn\">LinearFn</a>&gt;,\n    T: <a class=\"trait\" href=\"palette/trait.Component.html\" title=\"trait palette::Component\">Component</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,</div>"],["impl&lt;T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a> for <a class=\"struct\" href=\"palette/struct.Oklab.html\" title=\"struct palette::Oklab\">Oklab</a>&lt;T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,</div>"],["impl&lt;T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;T&gt; for <a class=\"struct\" href=\"palette/struct.Oklab.html\" title=\"struct palette::Oklab\">Oklab</a>&lt;T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,</div>"],["impl&lt;T: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a>, C: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;T&gt;&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;T&gt; for <a class=\"struct\" href=\"palette/struct.Alpha.html\" title=\"struct palette::Alpha\">Alpha</a>&lt;C, T&gt;"],["impl&lt;T: <a class=\"trait\" href=\"palette/float/trait.Float.html\" title=\"trait palette::float::Float\">Float</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>, C: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;T&gt;&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;T&gt; for <a class=\"struct\" href=\"palette/blend/struct.PreAlpha.html\" title=\"struct palette::blend::PreAlpha\">PreAlpha</a>&lt;C, T&gt;"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a> for <a class=\"struct\" href=\"palette/struct.Lab.html\" title=\"struct palette::Lab\">Lab</a>&lt;Wp, T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a>,</div>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a> for <a class=\"struct\" href=\"palette/struct.Luv.html\" title=\"struct palette::Luv\">Luv</a>&lt;Wp, T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a>,</div>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a> for <a class=\"struct\" href=\"palette/struct.Xyz.html\" title=\"struct palette::Xyz\">Xyz</a>&lt;Wp, T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a>,</div>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a> for <a class=\"struct\" href=\"palette/struct.Yxy.html\" title=\"struct palette::Yxy\">Yxy</a>&lt;Wp, T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a>,</div>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;T&gt; for <a class=\"struct\" href=\"palette/struct.Lab.html\" title=\"struct palette::Lab\">Lab</a>&lt;Wp, T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a>,</div>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;T&gt; for <a class=\"struct\" href=\"palette/struct.Luv.html\" title=\"struct palette::Luv\">Luv</a>&lt;Wp, T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a>,</div>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;T&gt; for <a class=\"struct\" href=\"palette/struct.Xyz.html\" title=\"struct palette::Xyz\">Xyz</a>&lt;Wp, T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a>,</div>"],["impl&lt;Wp, T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;T&gt; for <a class=\"struct\" href=\"palette/struct.Yxy.html\" title=\"struct palette::Yxy\">Yxy</a>&lt;Wp, T&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"palette/trait.FloatComponent.html\" title=\"trait palette::FloatComponent\">FloatComponent</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>,\n    Wp: <a class=\"trait\" href=\"palette/white_point/trait.WhitePoint.html\" title=\"trait palette::white_point::WhitePoint\">WhitePoint</a>,</div>"]]],["sk6812_rpi",[["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a> for <a class=\"struct\" href=\"sk6812_rpi/led/struct.Led.html\" title=\"struct sk6812_rpi::led::Led\">Led</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.83.0/std/primitive.f32.html\">f32</a>&gt; for <a class=\"struct\" href=\"sk6812_rpi/led/struct.Led.html\" title=\"struct sk6812_rpi::led::Led\">Led</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.83.0/core/ops/arith/trait.DivAssign.html\" title=\"trait core::ops::arith::DivAssign\">DivAssign</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.83.0/std/primitive.u8.html\">u8</a>&gt; for <a class=\"struct\" href=\"sk6812_rpi/led/struct.Led.html\" title=\"struct sk6812_rpi::led::Led\">Led</a>"]]]]);
    if (window.register_implementors) {
        window.register_implementors(implementors);
    } else {
        window.pending_implementors = implementors;
    }
})()
//{"start":57,"fragment_lengths":[13774,1043]}